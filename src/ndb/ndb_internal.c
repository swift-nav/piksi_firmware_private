/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/edc.h>
#include "libsbp/piksi.h"
#include "version.h"
#include "timing.h"
#include "ndb.h"
#include "ndb_internal.h"
#include "ndb_fs_access.h"
#include <nap/nap_common.h>
#include <signal.h>

#define NDB_THREAD_PRIORITY (LOWPRIO)
static WORKING_AREA_CCM(ndb_thread_wa, 2048);
static void ndb_service_thread(void*);

u8 ndb_file_version[MAX_NDB_FILE_VERSION_LEN];
u8 ndb_file_end_mark = 0xb6;

static MUTEX_DECL(data_access);

static ndb_element_metadata_t *wq_first = NULL;
static ndb_element_metadata_t *wq_last = NULL;

static ndb_element_metadata_t *ndb_wq_get(void);

static ndb_op_code_t ndb_create_file(const ndb_file_t *file);
static ndb_op_code_t ndb_open_file(const ndb_file_t *file);
static ndb_op_code_t ndb_read(const ndb_file_t *f, off_t o,
                              void *data, size_t s);
static ndb_op_code_t ndb_wq_process(void);

/**
 * Schedules metadata block for write processing.
 *
 * When scheduled, the block is appropriately marked.
 *
 * The method does nothing if the metadata block is already scheduled for
 * writing.
 *
 * \param[in] md Metadata block to schedule.
 *
 * \return None
 *
 * \sa ndb_wq_get
 */
void ndb_wq_put(ndb_element_metadata_t *md)
{
  assert(0 != (md->vflags & (NDB_VFLAG_IE_DIRTY | NDB_VFLAG_MD_DIRTY)));

  /* Check if the element is already in the queue. */
  if (0 == (md->vflags & NDB_VFLAG_ENQUEUED)) {
    if (NULL == wq_last) {
      wq_first = wq_last = md;
    } else {
      wq_last->next = md;
      wq_last = md;
    }
    md->next = NULL;
    /* Mark element as enqueued */
    md->vflags |= NDB_VFLAG_ENQUEUED;
  }
}

/**
 * Retrieves next metadata block scheduled for writing
 *
 * When retrieved, the block is appropriately marked.
 *
 * \return Next block pointer, or NULL if the queue is empty.
 */
static ndb_element_metadata_t *ndb_wq_get(void)
{
  ndb_element_metadata_t *res = wq_first;
  if (NULL != res) {
    wq_first = res->next;

    if (NULL == wq_first) {
      wq_last = NULL;
    }

    res->next = NULL;
  }
  if (NULL != res) {
    /* Reset queue flag to be symmetric with ndb_wq_put() */
    res->vflags &= ~NDB_VFLAG_ENQUEUED;
  }
  return res;
}

/**
 * NDB subsystem initialization.
 *
 * The method initializes NDB variables and computes supported file version.
 *
 * \sa ndb_start
 */
void ndb_init(void)
{
  if (!ndb_fs_is_real()) {
    log_info("NDB: configured not to save data to flash file system");
  }

  u16 vlen = sizeof(GIT_VERSION);
  vlen = vlen > sizeof(ndb_file_version) ? sizeof(ndb_file_version) : vlen;
  memcpy(ndb_file_version, GIT_VERSION, vlen);
  if (vlen < sizeof(ndb_file_version)) {
    memset(&ndb_file_version[vlen], 0xb6, sizeof(ndb_file_version) - vlen);
  }
}

/**
 * Starts up NDB operation.
 *
 * \sa ndb_init
 */
void ndb_start(void)
{
  chThdCreateStatic(ndb_thread_wa, sizeof(ndb_thread_wa),
                    NDB_THREAD_PRIORITY, ndb_service_thread, NULL);
}

/**
 * Helper to log file open operation status.
 *
 * \param[in] oc        Operation result.
 * \param[in] file_type File type.
 * \param[in] loaded    Total number of entries with contents.
 * \param[in] errors    Total number of error entries.
 *
 * \return None
 */
static void ndb_log_file_open(ndb_op_code_t oc,
                              const char *file_type,
                              u32 loaded,
                              u32 errors)
{
  switch (oc) {
  case NDB_ERR_NONE:
    log_info("NDB %s: File opened; loaded: %" PRIu32
             " entries, errors: %" PRIu32,
             file_type, loaded, errors);
    break;
  case NDB_ERR_FILE_IO:
    log_error("NDB %s: Can't open file", file_type);
    break;
  case NDB_ERR_INIT_DONE:
    log_info("NDB %s: Created empty file", file_type);
    break;
  case NDB_ERR_NO_CHANGE:
    log_info("NDB %s: Data has not been changed", file_type);
    break;
  case NDB_ERR_MISSING_IE:
  case NDB_ERR_UNSUPPORTED:
  case NDB_ERR_BAD_PARAM:
  case NDB_ERR_UNCONFIRMED_DATA:
  case NDB_ERR_ALGORITHM_ERROR:
  case NDB_ERR_NO_DATA:
  case NDB_ERR_OLDER_DATA:
  case NDB_ERR_AGED_DATA:
  case NDB_ERR_GPS_TIME_MISSING:
  default:
    assert(!"ndb_log_file_open()");
    break;
  }
}

/**
 * Performs initial load of NDB file contents.
 *
 * The method batch load NDB file data and metadata elements. Prior to the
 * load, the method checks the file existence and version match. If the file
 * doesn't exist, or the version doesn't math, the new file is created.
 *
 * When loading individual data blocks, each of them is checked for CRC
 * consistency, and blocks with wrong CRC are zeroed.
 *
 * \param[in,out]  file      NDB file to load
 * \param[in]      erase     When set to true, the data will be erased
 *
 * \return None
 */
void ndb_load_data(ndb_file_t *file, bool erase)
{
  ndb_op_code_t r = NDB_ERR_FILE_IO;
  size_t ds = (size_t)file->block_size * file->block_count;
  size_t mds = sizeof(ndb_element_metadata_nv_t) * file->block_count;
  ndb_element_metadata_nv_t md_nv[file->block_count];
  u8 *data = file->block_data;
  ndb_element_metadata_t *metadata = file->block_md;
  u32 loaded = 0, errors = 0;

  if (!erase) {
    /* Try to load data */
    r = ndb_open_file(file);
    if (NDB_ERR_NONE != r) {
      /* On error - request new file construction */
      erase = true;
    }
  }
  if (erase) {
    /* Create new file */
    r = ndb_create_file(file);
    if (NDB_ERR_NONE != r) {
      log_error("NDB %s: error %d while creating new file",
                file->type, (int)r);
    }
  }

  if (!erase && NDB_ERR_NONE == r) {
    r = ndb_read(file, 0, data, ds);
    if (NDB_ERR_NONE != r) {
      log_error("NDB %s: error %d while loading data",
                file->type, (int)r);
    } else {
      r = ndb_read(file, ds, md_nv, mds);
      if (NDB_ERR_NONE != r) {
        log_error("NDB %s: error %d while loading metadata",
                  file->type, (int)r);
      }
    }
  }

  if (!erase && NDB_ERR_NONE == r) {
    u8 *ptr = data;
    for (size_t i = 0; i < file->block_count;
         i++, ptr = ptr + file->block_size) {
      metadata[i].nv_data = md_nv[i];

      if (0 != (metadata[i].nv_data.state & NDB_IE_VALID)) {
        /* Set the flag indicating that the data is loaded from NV. */
        metadata[i].vflags |= NDB_VFLAG_DATA_FROM_NV;
        loaded++;
      }

      /* Check CRC: each block is protected with CRC24Q algorithm. The CRC
       * protection includes both block data and metadata sections */
      u32 old_crc = (u32)metadata[i].nv_data.crc[0] << 16;
      old_crc |= (u32)metadata[i].nv_data.crc[1] << 8;
      old_crc |= (u32)metadata[i].nv_data.crc[2] << 0;

      u32 new_crc = crc24q((const u8*)&metadata[i].nv_data,
                           sizeof(metadata[i].nv_data) -
                           sizeof(metadata[i].nv_data.crc),
                           crc24q(ptr, file->block_size, 0));

      if (old_crc != new_crc) {
        errors ++;

        log_warn("NDB %s: entry #%" PRIu32 " dropped", file->type, (u32)i);

        memset(ptr, 0, file->block_size);
        memset(&metadata[i].nv_data, 0, sizeof(metadata[i].nv_data));

        ndb_write_file_data(file, file->block_size * i + 0, ptr, file->block_size);
        ndb_write_file_data(file, sizeof(metadata[i].nv_data) * i + ds,
                            (u8 *)&metadata[i].nv_data, sizeof(metadata[i].nv_data));
      }
    }

    r = NDB_ERR_NONE;
  }

  if (NDB_ERR_NONE != r) {
    /* Initialize data element and metadata to zeros; this is required as the
     * data can be partially loaded. */
    memset(data, 0, ds);
    memset(metadata, 0, sizeof(ndb_element_metadata_t) * file->block_count);

    r = ndb_create_file(file);
    if (NDB_ERR_NONE == r) {
      r = NDB_ERR_INIT_DONE;
    }
  }

  ndb_log_file_open(r, file->type, loaded, errors);

  /* Initialize volatile metadata blocks */
  for (ndb_ie_index_t i = 0; i < file->block_count; i++) {
    metadata[i].data = data + (size_t)i * file->block_size;
    metadata[i].index = i;
    metadata[i].file = file;
    metadata[i].next = NULL;
  }
}

/**
 * Returns NAP time.
 *
 * \return NAP time in seconds
 */
ndb_timestamp_t ndb_get_timestamp(void)
{
  return nap_count_to_ms(nap_timing_count()) / 1000;
}

/**
 * Returns GPS time if available.
 *
 * \return GPS time in seconds
 */
gps_time_t ndb_get_GPS_timestamp(void)
{
  return (TIME_FINE == time_quality) ? napcount2rcvtime(nap_timing_count())
                                     : GPS_TIME_UNKNOWN;
}

/**
 * NDB service thread worker.
 *
 * The worker checks for incoming write requests, execute them, and report
 * results though SBP if appropriate.
 *
 * \param[in] p Thread parameter (unused)
 *
 * \return None
 */
static void ndb_service_thread(void *p)
{
  (void) (p);
  chRegSetThreadName("ndb");

  chThdSleepMilliseconds(NV_WRITE_REQ_TIMEOUT);
  while (true) {
    chThdSleepMilliseconds(NV_WRITE_REQ_TIMEOUT);
    while (true) {
      enum ndb_op_code res = ndb_wq_process();
      if (NDB_ERR_NO_DATA == res) {
        break;
      }
      if (NDB_ERR_NONE != res) {
        log_error("NDB Error writing data to the NDB file: code=%d", (int)res);
      }
    }
    ndb_sbp_updates();
  }
}

/**
 * Method for handling block save operation.
 *
 * The method fetches block data from a queue, computes CRC for it and
 * forwards the block data (optionally) and block metadata into persistence
 * layer.
 *
 * \retval NDB_ERR_NONE     On success
 * \retval NDB_ERR_NO_DATA  No more data to process
 * \retval NDB_ERR_FILE_IO  On file I/O error
 */
static ndb_op_code_t ndb_wq_process(void)
{
  ndb_element_metadata_t *md = NULL;

  /* Lock NDB and fetch block data and metadata to write along with flags */
  ndb_lock();
  md = ndb_wq_get();
  if (NULL == md) {
    /* No data to process */
    ndb_unlock();
    return NDB_ERR_NO_DATA;
  }

  /* Locally load information from the dequeued element */
  ndb_ie_size_t              block_size = md->file->block_size;
  ndb_element_metadata_nv_t  nv_data;
  u8                         buf[block_size];
  bool                       write_buf = false;
  ndb_file_t                *file =  md->file;
  ndb_ie_index_t             index = md->index;

  assert((md->vflags & (NDB_VFLAG_IE_DIRTY | NDB_VFLAG_MD_DIRTY)) != 0);
  assert((md->vflags & NDB_VFLAG_ENQUEUED) == 0);

  memcpy(buf, md->data, block_size);
  write_buf = 0 != (md->vflags & NDB_VFLAG_IE_DIRTY);

  /* Mark as saved (optimistically) */
  md->vflags &= ~(NDB_VFLAG_IE_DIRTY | NDB_VFLAG_MD_DIRTY);
  nv_data = md->nv_data;

  ndb_unlock();

  ndb_op_code_t ret = NDB_ERR_NONE;

  if (write_buf) {
    ret = ndb_write_file_data(file, (u32)block_size * index, buf, block_size);
  }
  if (NDB_ERR_NONE == ret) {
    /* Compute CRC-24Q: the check sum includes data block + metadata block */
    u32 crc = crc24q((const u8 *)&nv_data,
                     sizeof(nv_data) - sizeof(nv_data.crc),
                     crc24q(buf, block_size, 0));
    nv_data.crc[0] = (u8)(crc >> 16);
    nv_data.crc[1] = (u8)(crc >> 8);
    nv_data.crc[2] = (u8)(crc);

    log_debug("NDB store %s[%" PRIu32 "] crc=0x%06"PRIX32,
              file->name, (u32)index, crc);

    off_t offset = ((u32)block_size * file->block_count +
                    sizeof(ndb_element_metadata_nv_t) * index);
    ret = ndb_write_file_data(file, offset, (u8 *)&nv_data, sizeof(nv_data));
  }
  return ret;
}

/**
 * Tests if the given version matches the supported one.
 *
 * At the moment the version must match the version generated during the build
 * process. There is no cross-build version portability.
 *
 * \param[in] version Version to check
 *
 * \retval true  Version matches
 * \retval false Version differs
 */
static bool ndb_file_verion_match(const char version[MAX_NDB_FILE_VERSION_LEN])
{
  if (ndb_fs_is_real()) {
    return memcmp(version, ndb_file_version, sizeof(ndb_file_version)) == 0;
  } else {
    return true;
  }
}

/**
 * Computes NDB file size in bytes.
 *
 * NDB file contains:
 * - NDB file version (prefix)
 * - NDB `file->block_count` elements of `file->block_size` size
 * - NDB `file->block_count` `ndb_element_metadata_nv_t` elements
 * - NDB file end mark (suffix)
 *
 * \param[in] file NDB file descriptor
 *
 * \return NDB file size in bytes
 */
static inline size_t ndb_compute_size(const ndb_file_t *file)
{
  return sizeof(ndb_file_version) + sizeof(ndb_file_end_mark) +
         ((size_t)file->block_size + sizeof(ndb_element_metadata_nv_t)) *
         file->block_count;
}

/**
 * Creates new empty file.
 *
 * \param[in] file   NDB file
 * \param[in] create Flag, if NDB data has to be erased
 *
 * \retval NDB_ERR_NONE    On success
 * \retval NDB_ERR_FILE_IO On I/O error
 *
 * \sa ndb_open_file
 */
static ndb_op_code_t ndb_create_file(const ndb_file_t *file)
{
  size_t file_size = ndb_compute_size(file);

  log_info("NDB %s: creating new empty file; size=%" PRIu32,
           file->type, (u32)file_size);

  /* Delete file if it exists */
  ndb_fs_remove(file->name);
  char ver[MAX_NDB_FILE_VERSION_LEN];
  ssize_t read_res = ndb_fs_read(file->name, 0, ver, sizeof(ver));
  if (read_res > 0) {
    log_warn("NDB %s: failed to remove file", file->type);
  }

  /* Create empty file (all zeros) */
  ndb_fs_reserve(file->name, file_size);

  /* Write file version */
  if (ndb_fs_write(file->name, 0, ndb_file_version, sizeof(ndb_file_version))
      != sizeof(ndb_file_version)) {
    return NDB_ERR_FILE_IO;
  }
  /* Write file end mark */
  if (ndb_fs_write(file->name,
                   file_size - sizeof(ndb_file_end_mark),
                   &ndb_file_end_mark,
                   sizeof(ndb_file_end_mark)) != sizeof(ndb_file_end_mark)) {
    return NDB_ERR_FILE_IO;
  }

  return ndb_open_file(file);
}


/**
 * Opens NDB file that stores information elements of certain type.
 * This function checks if version of the file matches the passed one
 * and if it doesn't creates empty file automatically.
 *
 * \param[in] file  NDB file
 *
 * \retval NDB_ERR_NONE    On success
 * \retval NDB_ERR_NO_DATA If the file is missing
 * \retval NDB_ERR_FILE_IO On I/O error
 *
 * \sa ndb_create_file
 */
static ndb_op_code_t ndb_open_file(const ndb_file_t *file)
{
  size_t expected_size = sizeof(ndb_file_version) + sizeof(ndb_file_end_mark) +
                         ((size_t)file->block_size +
                          sizeof(ndb_element_metadata_nv_t)) * file->block_count;

  /* Check file version */
  char ver[MAX_NDB_FILE_VERSION_LEN];
  ssize_t read_res = ndb_fs_read(file->name, 0, ver, sizeof(ver));
  if (read_res < 0) {
    log_warn("NDB %s: failed to read header", file->type);
  } else if (0 == read_res) {
    /* EOF - file is not present */
    return NDB_ERR_NO_DATA;
  } else if (sizeof(ver) == read_res) {
    if (ndb_file_verion_match(ver)) {
      /* Check end mark */
      u8   em;
      read_res = ndb_fs_read(file->name,
                             expected_size - sizeof(em),
                             &em, sizeof(em));
      if (read_res < 0) {
        log_warn("NDB %s: failed to read end mark", file->type);
      } else if (read_res == sizeof(em)) {
        if (em == ndb_file_end_mark) {
          /* Both version end end marks are OK */
          return NDB_ERR_NONE;
        } else {
          log_warn("NDB %s: end mark is incorrect", file->type);
        }
      } else {
        log_warn("NDB %s: end mark length is incorrect", file->type);
      }
    } else {
      log_info("NDB %s: file version mismatch; new file is created", file->type);
    }
  } else {
    log_warn("NDB %s: version length is incorrect; "
             "expected=%" PRId32 " read=%" PRId32,
             file->type, (s32)sizeof(ver), (s32)read_res);
  }

  return NDB_ERR_FILE_IO;
}

/**
 * Writes data block into NDB file.
 *
 * \param[in] file  NDB file
 * \param[in] off   Block offset in bytes
 * \param[in] src   Data write buffer
 * \param[in] size  Size of the data block to read
 *
 * \retval NDB_ERR_NONE    On successful read.
 * \retval NDB_ERR_FILE_IO On read error.
 *
 * \sa ndb_read
 */
ndb_op_code_t ndb_write_file_data(ndb_file_t *file,
                                  off_t off,
                                  const u8 *src,
                                  size_t size)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  off_t   offset = sizeof(ndb_file_version) + off;
  ssize_t written = ndb_fs_write(file->name, offset, src, size);
  if (written < 0) {
    log_warn("NDB %s: write error", file->type);
    res = NDB_ERR_FILE_IO;
  } else if ((size_t)written != size) {
    log_warn("NDB %s: incorrect write result; expected=%" PRId32
             " actual=%" PRId32,
             file->type, (s32)size, (s32)written);
    res = NDB_ERR_FILE_IO;
  } else {
    res = NDB_ERR_NONE;
  }

  return res;
}

/**
 * Reading from NDB file.
 * This function is to be called only during initialization of NDB.
 * It is intended to read all information elements collection from
 * file into memory.
 *
 * \param[in]  file  NDB file
 * \param[in]  off   Block offset in bytes
 * \param[out] dst   Buffer for data read
 * \param[in]  size  Size of the data block to read
 *
 * \retval NDB_ERR_NONE    On successful read.
 * \retval NDB_ERR_FILE_IO On read error.
 *
 * \sa ndb_write_file_data
 */
static ndb_op_code_t ndb_read(const ndb_file_t *file,
                              off_t off,
                              void *dst,
                              size_t size)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;
  off_t offset = sizeof(ndb_file_version) + off;
  ssize_t read =  ndb_fs_read(file->name, offset, dst, size);

  if (read < 0) {
    log_warn("NDB %s: read error", file->type);
    res = NDB_ERR_FILE_IO;
  } else if ((size_t)read != size) {
    log_warn("NDB %s: incorrect read result; expected=%" PRId32
             " actual=%" PRId32,
             file->type, (s32)size, (s32)read);
    res = NDB_ERR_FILE_IO;
  } else {
    res = NDB_ERR_NONE;
  }

  return res;
}

/**
 * Locks NDB database state.
 *
 * \sa ndb_unlock
 */
void ndb_lock()
{
  chMtxLock(&data_access);
}

/**
 * Unlocks NDB database state.
 *
 * \sa ndb_unlock
 */
void ndb_unlock()
{
  chMtxUnlock(&data_access);
}

/**
 * Internal data retrieval function
 *
 * \param[in]  file     NDB file object
 * \param[in]  idx      NDB informational element index
 * \param[out] out      Destination data buffer with a proper block size.
 * \param[out] ds       Optional destination for NDB data source.
 *
 * \retval NDB_ERR_NONE       On success
 * \retval NDB_ERR_BAD_PARAM  On parameter error
 * \retval NDB_ERR_MISSING_IE No cached data block
 *
 * \internal
 */
static ndb_op_code_t ndb_retrieve_int(ndb_file_t *file,
                                      ndb_ie_index_t idx,
                                      void *out,
                                      ndb_data_source_t *ds)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  assert(idx < file->block_count);

  const ndb_element_metadata_t *md = &file->block_md[idx];

  if (0 != (md->nv_data.state & NDB_IE_VALID)) {
    memcpy(out, md->data, md->file->block_size);

    if (NULL != ds) {
      *ds = md->nv_data.source;
    }

    res = NDB_ERR_NONE;
  } else {
    res = NDB_ERR_MISSING_IE;
    /* Zero destination */
    memset(out, 0, md->file->block_size);
    if (NULL != ds) {
      *ds = NDB_DS_UNDEFINED;
    }
  }
  return res;
}

/**
 * Reads NDB data block if found.
 *
 * Method provides NDB data block contents that is located using match function.
 *
 * \param[in]  file     NDB file object
 * \param[in]  match_fn NDB informational element match function
 * \param[in]  cookie   Parameter for \a match_fn
 * \param[out] out      Destination data buffer with a proper block size.
 * \param[in]  out_size Destination buffer size. Must match block size defined
 *                      in file metadata section.
 * \param[out] ds       Optional destination for NDB data source.
 *
 * \retval NDB_ERR_NONE       On success
 * \retval NDB_ERR_BAD_PARAM  On parameter error
 * \retval NDB_ERR_MISSING_IE No cached data block
 *
 * \sa ndb_update
 * \sa ndb_retrieve
 */
ndb_op_code_t ndb_find_retrieve(ndb_file_t *file,
                                ndb_entry_match_fn match_fn,
                                void *cookie,
                                void *out,
                                size_t out_size,
                                ndb_data_source_t *ds)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  if (NULL != file && NULL != out && NULL != match_fn &&
      out_size == file->block_size) {

    ndb_lock();
    res = NDB_ERR_MISSING_IE;
    u8 *data_ptr = file->block_data;
    for (ndb_ie_index_t idx = 0; idx < file->block_count;
         ++idx, data_ptr += file->block_size) {
      if (match_fn(data_ptr, &file->block_md[idx], cookie)) {
        res = ndb_retrieve_int(file, idx, out, ds);
        break;
      }
    }
    ndb_unlock();
  } else {
    res = NDB_ERR_BAD_PARAM;
  }

  return res;
}

/**
 * Reads NDB data block.
 *
 * Method provides NDB data block contents.
 *
 * \param[in]  md       Metadata element
 * \param[out] out      Destination data buffer with a proper block size.
 * \param[in]  out_size Destination buffer size. Must match block size defined
 *                      in file metadata section.
 * \param[out] ds       Optional destination for NDB data source.
 * \param[in]  use_nv   Flag indicating if data loaded from NV should be used.
 *
 * \retval NDB_ERR_NONE       On success
 * \retval NDB_ERR_BAD_PARAM  On parameter error
 * \retval NDB_ERR_MISSING_IE No cached data block
 *
 * \sa ndb_update
 * \sa ndb_file_retrieve
 */
ndb_op_code_t ndb_retrieve(const ndb_element_metadata_t *md,
                           void *out,
                           size_t out_size,
                           ndb_data_source_t *ds,
                           bool use_nv)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  bool use_valid = true;
  /* If data has been load from NV and data from NV should not be used,
   * then mark use_valid false. */
  if (!use_nv && (md->vflags & NDB_VFLAG_DATA_FROM_NV) != 0) {
    use_valid = false;
  }

  if (NULL != md && NULL != md->file && out_size == md->file->block_size
      && use_valid) {
    ndb_lock();
    res = ndb_retrieve_int(md->file, md->index, out, ds);
    ndb_unlock();
  } else {
    res = NDB_ERR_BAD_PARAM;
  }

  return res;
}

/**
 * Update NDB data block.
 *
 * The method updates NDB data block, marks it valid and updates block metadata.
 * After an update, the block is scheduled for asynchronous write.
 *
 * \param[in]     data New block data.
 * \param[in]     src  Block data source.
 * \param[in,out] md   Block metadata.
 *
 * \retval NDB_ERR_NONE      On success
 * \retval NDB_ERR_BAD_PARAM On parameter error
 *
 * \sa ndb_retrieve
 * \sa ndb_erase
 */
ndb_op_code_t ndb_update(const void *data,
                         ndb_data_source_t src,
                         ndb_element_metadata_t *md)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  if (NULL != data && NULL != md) {
    ndb_ie_size_t block_size = md->file->block_size;

    ndb_lock();

    /* Update metadata and mark it dirty */
    md->vflags |= NDB_VFLAG_MD_DIRTY;
    md->nv_data.source = src;

    if (memcmp(data, md->data, block_size) != 0 ||
        (md->vflags & NDB_VFLAG_DATA_FROM_NV) != 0) {
      /* If data has been originally loaded from NV
       * then clear the flag here. */
      md->vflags &= ~NDB_VFLAG_DATA_FROM_NV;
      /* Update data and mark it dirty also mark data valid */
      memcpy(md->data, data, block_size);
      md->nv_data.state |= NDB_IE_VALID;
      md->vflags |= NDB_VFLAG_IE_DIRTY;
      res = NDB_ERR_NONE;
    } else {
      /* data we try to write to NDB is the same as previously stored */
      res = NDB_ERR_NO_CHANGE;
    }

    ndb_wq_put(md);
    ndb_unlock();

  } else {
    res = NDB_ERR_BAD_PARAM;
  }

  return res;
}

/**
 * Erase NDB data block.
 *
 * After erasing, NDB block is zeroed, data source is set to #NDB_DS_UNDEFINED
 * and data validity flag is cleared.
 *
 * \param[in,out] md NDB data block metadata
 *
 * \retval NDB_ERR_NONE      On success
 * \retval NDB_ERR_NO_CHANGE On success, but no data has been present.
 * \retval NDB_ERR_BAD_PARAM On parameter error
 *
 * \sa ndb_update
 */
ndb_op_code_t ndb_erase(ndb_element_metadata_t *md)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  if (NULL != md) {
    ndb_ie_size_t block_size = md->file->block_size;

    ndb_lock();

    bool md_modified = false;
    if (NDB_DS_UNDEFINED != md->nv_data.source) {
      /* Update metadata and mark it dirty */
      md->nv_data.source = NDB_DS_UNDEFINED;
      md->vflags |= NDB_VFLAG_MD_DIRTY;
      md_modified = true;
    }

    if (0 != (md->nv_data.state & NDB_IE_VALID)) {
      /* Update data and mark it dirty also mark data invalid */
      memset(md->data, 0, block_size);
      md->nv_data.state &= ~NDB_IE_VALID;
      md->vflags |= NDB_VFLAG_IE_DIRTY | NDB_VFLAG_MD_DIRTY;
      md_modified = true;
    }

    if (md_modified) {
      ndb_wq_put(md);
      res = NDB_ERR_NONE;
    } else {
      res = NDB_ERR_NO_CHANGE;
    }
    ndb_unlock();

  } else {
    res = NDB_ERR_BAD_PARAM;
  }

  return res;
}

/** Determine next signal index data to be sent over SBP.
 *  This function takes previous index (set to NDB_SBP_UPDATE_SIG_IDX_INIT to
 *  indicate no previous value available) and increments it by one making sure
 *  that index is within codes that contain 'original' ephemerides. This is
 *  necessary to prevent outputting the same ephemeris for different codes
 *  of the same satellite.
 *  */
static u32 get_next_idx_to_send(gnss_signal_t *sid, s32 prev_idx)
{
  u32 i = prev_idx != NDB_SBP_UPDATE_SIG_IDX_INIT ? prev_idx + 1 : 0;

  while (i < PLATFORM_SIGNAL_COUNT) {
    *sid = sid_from_global_index(i);
    if (sid->code != CODE_GPS_L1CA &&
        sid->code != CODE_SBAS_L1CA &&
        sid->code != CODE_GLO_L1CA) {
      i++;
    } else {
      break;
    }
  }

  return i;
}

void ndb_sbp_update(ndb_sbp_update_info_t *info)
{
  /* increment call counter */
  info->count++;

  /* check epoch cycle and if all signals are handled */
  bool epoch_cycle_full = (info->count >= info->epoch_spacing &&
                           info->sig_idx == PLATFORM_SIGNAL_COUNT);

  if (epoch_cycle_full) {
    /* reinit values */
    info->count = NDB_SBP_UPDATE_CYCLE_COUNT_INIT;
    info->sig_idx = NDB_SBP_UPDATE_SIG_IDX_INIT;
    /* return to make sure msg spacing is cycled */
    return;
  }

  /* check if all valid signals are already sent for this epoch */
  bool tx_enabled = (info->sig_idx != PLATFORM_SIGNAL_COUNT);

  if (tx_enabled) {
    /* is it time to send next msg */
    bool cycle_full = (0 == info->count % info->msg_spacing);
    if (cycle_full) {
      gnss_signal_t sid;
      info->sig_idx = get_next_idx_to_send(&sid, info->sig_idx);

      /* do until valid data is sent or we reach the end of signal array */
      while (info->sig_idx < PLATFORM_SIGNAL_COUNT) {
        if (info->tx_func(sid)) {
          /* msg sent */
          return;
        }
        info->sig_idx = get_next_idx_to_send(&sid, info->sig_idx);
      }
    }
  }
}

