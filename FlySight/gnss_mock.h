/***************************************************************************
**  FlySight 2 firmware — GPS mock for SD-card replay                     **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
****************************************************************************/

#ifndef GNSS_MOCK_H_
#define GNSS_MOCK_H_

/*
 * Set GNSS_MOCK_ENABLED to 1 to replay GPS data from MOCK.CSV on the SD card
 * instead of reading from the real GNSS hardware.  All consumers of
 * FS_GNSS_GetData() and the data-ready callback behave identically.
 *
 * MOCK.CSV must be placed in the root of the SD card.  See gnss_mock.c for
 * the expected CSV column layout.
 */
#define GNSS_MOCK_ENABLED  0

#if GNSS_MOCK_ENABLED

/*
 * Call once during startup, after the SD card is mounted.
 * Opens MOCK.CSV, reads the first record, and starts the replay timer.
 * If the file cannot be opened the mock silently does nothing.
 */
void FS_GNSS_Mock_Init(void);

/* Stop replay and release the SD card file handle. */
void FS_GNSS_Mock_DeInit(void);

#endif /* GNSS_MOCK_ENABLED */

#endif /* GNSS_MOCK_H_ */
