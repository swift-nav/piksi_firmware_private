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

#define MAX_NDB_FILE_VERSION_LEN 64
extern u8 ndb_file_version[MAX_NDB_FILE_VERSION_LEN];

/* Information element size */
typedef u16 ndb_ie_size_t;
/* Information element index in the array */
typedef u8 ndb_ie_index_t;

/** NDB File */
typedef struct __attribute__((packed)) {
  const char *name;        /**< Name of the file */
  const u32 expected_size; /**< Expected file size */
  const u16 data_size;     /**< Size of data element */
  const u16 n_elements;    /**< Number of data elements */
} ndb_file_t;

/** Maximum waiting time for write request, milliseconds */
#define NV_WRITE_REQ_TIMEOUT 100

/* TODO separate persistent and non-persistent NDB flags */
/** IE needs to be written to NVM */
#define NDB_IE_DIRTY (1 << 0)
/** Value has been set */
#define NDB_IE_VALID (1 << 1)
/** Metadata needs to be written to NVM */
#define NDB_MD_DIRTY (1 << 2)
/** Data block is in write queue */
#define NDB_ENQUEUED (1 << 3)
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
  ndb_file_t                  *file;    /**< NDB file object pointer */
  struct ndb_element_metadata *next;    /**< Next element for operation queue */
} ndb_element_metadata_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

extern u8 ndb_file_end_mark;

void ndb_init(void);
void ndb_start(void);
void ndb_lock(void);
void ndb_unlock(void);

ndb_timestamp_t ndb_get_timestamp(void);
void ndb_load_data(ndb_file_t *f,
                   const char *ftype,
                   u8 *data,
                   ndb_element_metadata_t *metadata,
                   size_t el_size,
                   size_t el_number);
enum ndb_op_code ndb_update(const void *data,
                            enum ndb_data_source src,
                            ndb_element_metadata_t *md);
void ndb_retrieve(void *out, const void *cached, size_t size);
enum ndb_op_code ndb_write_file_data(ndb_file_t *file,
                                     off_t off,
                                     const u8 *src,
                                     size_t size);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SRC_NDB_INTERNAL_H_ */
