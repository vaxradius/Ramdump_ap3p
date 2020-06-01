#ifndef __AM_STORAGE_H
#define __AM_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Reads data from persistent storage.
 *
 * @param[in] address The address to start reading from.
 * @param[in] size The number of bytes to read.
 * @param[in,out] data The returned data. This memory must be allocated by
 * the caller.
 * @return :: 0 for success ; -1 for failed
 */
int8_t storage_read(uint32_t address, size_t size, void *data);

/**
 * @brief Writes data to persistent storage.
 *
 * @param[in] address The address to start writing to.
 * @param[in] data The data to be written.
 * @param[in] size The number of bytes to write.
 * @return :: 0 for success ; -1 for failed
 */
int8_t storage_write(uint32_t address, const void *data, size_t size);

/**
 * @brief Erases data from persistent storage.
 *
 * @param[in] address The address to start erasing from.
 * @param[in] size The number of bytes to erase.
 * @return :: 0 for success ; -1 for failed
 */
int8_t storage_erase(uint32_t address, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* __AM_STORAGE_H */

