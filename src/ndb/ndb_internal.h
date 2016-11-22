/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri.atamaniouk@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SRC_NDB_INTERNAL_H_
#define SRC_NDB_INTERNAL_H_

#include <libswiftnav/common.h>

#define MAX_NDB_FILE_VERSION_LEN 64
extern u8 ndb_file_version[MAX_NDB_FILE_VERSION_LEN];

/** Information element size */
typedef u16 ndb_ie_size_t;
/** Information element index in the array */
typedef u8 ndb_ie_index_t;
/** Information element volatile flags */
typedef u8 ndb_vflags_t;

/** NDB File */
typedef struct {
  const char                  * const name;        /**< Name of the file */
  const char                  * const type;        /**< Type of the file */
  u8                          * const block_data;  /**< IE data */
  struct ndb_element_metadata * const block_md;    /**< Metadata */
  const ndb_ie_size_t                 block_size;  /**< Size of data element */
  const ndb_ie_index_t                block_count; /**< Number of data elements */
} ndb_file_t;

/** Maximum waiting time for write request, milliseconds */
#define NV_WRITE_REQ_TIMEOUT 100

/** Volatile flag: IE needs to be written to NVM */
#define NDB_VFLAG_IE_DIRTY (1 << 0)
/** Volatile flag: Metadata needs to be written to NVM */
#define NDB_VFLAG_MD_DIRTY (1 << 1)
/** Volatile flag: Data block is in write queue */
#define NDB_VFLAG_ENQUEUED (1 << 2)

/** Non-volatile flag: value has been set */
#define NDB_IE_VALID (1 << 0)
/** Metadata has valid data */
#define NDB_IE_IS_VALID(md_ptr) (0 != ((md_ptr)->nv_data.state & NDB_IE_VALID))

/** Persistent NDB metadata block */
typedef struct __attribute__((packed)) ndb_element_metadata_nv
{
  ndb_timestamp_t received_at;  /**< TAI timestamp [s] */
  u8              source: 4;    /**< Data source */
  u8              state: 4;     /**< State flags */
  u8              crc[3];       /**< CRC-24Q */
} ndb_element_metadata_nv_t;

/** Non-persistent NDB metadata block */
typedef struct ndb_element_metadata
{
  ndb_element_metadata_nv_t    nv_data; /**< Persistent block */
  void                        *data;    /**< Data block pointer */
  ndb_ie_index_t               index;   /**< Index inside file */
  ndb_vflags_t                 vflags;  /**< Non-persistent flags */
  ndb_file_t                  *file;    /**< NDB file object pointer */
  struct ndb_element_metadata *next;    /**< Next element for operation queue */
} ndb_element_metadata_t;

/** Candidate update status */
typedef enum {
  NDB_CAND_IDENTICAL,     /**< No update, as the same data is already present */
  NDB_CAND_OLDER,         /**< No update, as newer data is already present */
  NDB_CAND_NEW_CANDIDATE, /**< New candidate accepted */
  NDB_CAND_NEW_TRUSTED,   /**< Previous candidate confirmed */
  NDB_CAND_MISMATCH,      /**< Candidate data mismatch */
} ndb_cand_status_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef bool ndb_entry_match_fn(const void *data,
                                const ndb_element_metadata_t *md,
                                void *cookie);

extern u8 ndb_file_end_mark;

void ndb_init(void);
void ndb_start(void);
void ndb_lock(void);
void ndb_unlock(void);

ndb_timestamp_t ndb_get_timestamp(void);
void ndb_load_data(ndb_file_t *f, bool erase);
ndb_op_code_t ndb_update(const void *data,
                         ndb_data_source_t src,
                         ndb_element_metadata_t *md);
ndb_op_code_t ndb_erase(ndb_element_metadata_t *md);
ndb_op_code_t ndb_retrieve(const ndb_element_metadata_t *md,
                           void *out,
                           size_t out_size,
                           ndb_data_source_t *src,
                           ndb_timestamp_t *ts);
ndb_op_code_t ndb_find_retrieve(ndb_file_t *file,
                                ndb_entry_match_fn match_fn,
                                void *cookie,
                                void *out,
                                size_t out_size,
                                ndb_data_source_t *src,
                                ndb_timestamp_t *ts);
ndb_op_code_t ndb_write_file_data(ndb_file_t *file,
                                  off_t off,
                                  const u8 *src,
                                  size_t size);
void ndb_wq_put(ndb_element_metadata_t *md);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SRC_NDB_INTERNAL_H_ */
