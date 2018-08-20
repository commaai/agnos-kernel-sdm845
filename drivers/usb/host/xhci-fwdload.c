/*
 * RENESAS USB 3.0 Host Controller uPD720201 and uPD720202
 * FW Download Solution for Linux
 *
 * Copyright (C) 2011 - 2014  Renesas Electronics Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
///////////////////////////////////////////////////////////////////////////////
//
//      History
//      2011-10-28 rev1.0     base create
//      2011-11-01 rev1.1     Reflect changed the wait timeout flag / status.
//      2011-11-17 rev1.2     Add to download FW return from suspend.
//      2011-11-21 rev1.3     When ReadModifyWrite the SetData1, SetData0 was to write a zero.
//                            SetData1 also made a similar action.
//      2011-11-22 rev1.4     Remove the code that was put to the test.
//      2011-11-22 rev1.5     Reflect changed the wait timeout flag / status.
//      2011-11-22 rev1.6     Remove the code that was put to the test.
//      2011-12-05 rev1.7     Change header comment and FW file name.
//                            To respond to the 64bit kernel, change the variables type "long" to "int".
//                            (variable of type long is different size 64bit kernel and 32bit kernel)
//      2012-04-10 rev1.8     Add processing for Rev.3 or later.
//      2014-01-06 rev2.0     Change the FW version
//                            Modify the processing for D720201 Rev3
//                            Modify the warning code
//                            
//
///////////////////////////////////////////////////////////////////////////////
#include <linux/time.h>
#include <linux/firmware.h>

#include "xhci.h"

//@@---------------------- PCI informarion
#define XHCI_FWFILENAME_720201_202ES20  "K2026090.mem"

#define XHCI_VENDOR_ID_720201           (0x1912)
#define XHCI_DEVICE_ID_720201           (0x0014)
#define XHCI_VENDOR_ID_720202           (0x1912)
#define XHCI_DEVICE_ID_720202           (0x0015)
#define XHCI_DEVICE_REV_ES12            (0x01)
#define XHCI_DEVICE_REV_ES20            (0x02)
#define XHCI_DEVICE_REV_CS              (0x03)

//@@---------------------- Registers for firmware downloading
#define PCICNF0F4  0xF4  // control and status bits
   /*
   0 = FW download enable (1), RW
   1 = FW download lock (1) or unlock (0), need 0 to perform download, RW(Write Once)
   6:4 = Result code, RO -- processing (0), success (1), error (2),
   8 = Set Data 0, RW
   9 = Set Data 1, RW
   16:31 = (used for serial EEPROM read/write.  31 = serial EEPROM present.)
   */
#define PCICNF0F4_FWDOWNLOADENABLE  (0x0001)
#define PCICNF0F4_FWDOWNLOADLOCK    (0x0002)
#define PCICNF0F4_SETDATA0          (0x0100)
#define PCICNF0F4_SETDATA1          (0x0200)
#define PCICNF0F4_RESULT            (0x0070)
#define PCICNF0F4_FWDOWNLOADENABLE_B    (0)
#define PCICNF0F4_FWDOWNLOADLOCK_B      (1)
#define PCICNF0F4_SETDATA0_B            (8)
#define PCICNF0F4_SETDATA1_B            (9)
#define PCICNF0F4_RESULT_B              (4)

#define PCICNF0F8  0xF8  // data 0 dword

#define PCICNF0FC  0xFC  // data 1 dword

//#define SINGLE_PAGE

//@@---------------------- Other define

typedef enum {
    XHCI_CONTROLREGISTER_BIT_FW_DOWNLOAD_ENABLE,
    XHCI_CONTROLREGISTER_BIT_FW_DOWNLOAD_LOCK,
    XHCI_CONTROLREGISTER_BIT_SET_DATA0,
    XHCI_CONTROLREGISTER_BIT_SET_DATA1,

    XHCI_CONTROLREGISTER_RESULT,

} XHCI_CONTROLREGISTER_BIT;

typedef enum {
    XHCI_FWDOWNLOADCONTROLBIT_DISABLE = 0,
    XHCI_FWDOWNLOADCONTROLBIT_ENABLE  = 1,

} XHCI_FWDOWNLOADCONTROLBIT_VALUE;

typedef enum {
    XHCI_FWDOWNLOAD_STATUS_INVALID = 0x00,
    XHCI_FWDOWNLOAD_STATUS_SUCCESS = 0x01,
    XHCI_FWDOWNLOAD_STATUS_ERROR   = 0x02,

} XHCI_FWDOWNLOAD_STATUS;

#define XHCI_CONTROL_TIMEOUT_1000MS   (1000)
#define XHCI_CONTROL_TIMEOUT_DEVIDE  (1000)
#define XHCI_CONTROL_TIMEOUT_NOSLEEP

//#define CONFIG_USB_XHCI_HCD_DEBUGGING
#ifdef CONFIG_USB_XHCI_HCD_DEBUGGING
#define DEBUG_FORMAT_MODULE_NAME    "[XHCI FW Downloder] "
#define XHCI_DebugPrint(fmt, args...)   printk(KERN_CRIT DEBUG_FORMAT_MODULE_NAME fmt "\n", ## args)
#else
#define XHCI_DebugPrint(fmt, args...)
#endif
#define DebugPrintFunctionName() XHCI_DebugPrint("%s\n", __FUNCTION__)

//#define XHCI_DEBUG_READ_ENABLE
//#define XHCI_DEBUG_WRITE_ENABLE
//#define XHCI_DEBUG_FLAG_ENABLE

//------------------------------------------------------------- CFR
static inline XHCI_FWDOWNLOADER cfgprd (
    struct pci_dev  *pDev,      // Target Device(720201/720202)
    unsigned int nOffset,      // Offset in the configuration space to read
    unsigned int *pReaddata    // Pointer to the buffer for storing read data
) {
   // Read a config reg dword using a PCI protocol pointer.
   // At present, this function is used only by FDL.

    if(pci_read_config_dword(pDev, (int)nOffset, (u32*)pReaddata) == 0) {
#ifdef XHCI_DEBUG_READ_ENABLE
        XHCI_DebugPrint("Read[%08x]->Data[%08x]", nOffset, *pReaddata);
#endif
        return XHCI_FWDOWNLOADER_SUCCESS;
    }
    else {
#ifdef XHCI_DEBUG_READ_ENABLE
        XHCI_DebugPrint("PCI Config Read  error(Offset=%08x)", nOffset);
#endif
        return XHCI_FWDOWNLOADER_ERROR;
    }
}

//----------------------------------------------------------------------- CFW
static inline XHCI_FWDOWNLOADER cfgpw (
    struct pci_dev  *pDev,      // Target Device(720201/720202)
    unsigned int nOffset,      // Offset in the configuration space to read
    unsigned int nWritedata    // WriteData(32bit)
) {
   // Write a config reg dword using a PCI protcol pointer.
   // At present, this function is used only by FDL.

    if(pci_write_config_dword(pDev, (int)nOffset, (u32)nWritedata) == 0) {
#ifdef XHCI_DEBUG_WRITE_ENABLE
        XHCI_DebugPrint("Write[%08x]<-Data[%08x]", nOffset, *pReaddata);
#endif
        return XHCI_FWDOWNLOADER_SUCCESS;
    }
    else {
#ifdef XHCI_DEBUG_WRITE_ENABLE
        XHCI_DebugPrint("PCI Config Write error(Offset=%08x,Data=%08x)", nOffset, nWritedata);
#endif
        return XHCI_FWDOWNLOADER_ERROR;
    }
}

//----------------------------------------------------------
static XHCI_FWDOWNLOADER XHCI_CheckControlWithTimeOut (
    struct pci_dev                  *pDev,      // Target Device(720201/720202)
    XHCI_CONTROLREGISTER_BIT        eBit,       // Bit name for check
    XHCI_FWDOWNLOADCONTROLBIT_VALUE eValue,     // value for check

    unsigned int                   nTimeOut    // Time out [ms]
) {
    // To check the status of a specified bit in eBit
    XHCI_FWDOWNLOADER result = XHCI_FWDOWNLOADER_ERROR;
    unsigned int mask = 0;
    unsigned int bit  = 0;

#ifdef XHCI_CONTROL_TIMEOUT_NOSLEEP
    struct timespec start;
    struct timespec now;
#else
    int timeout_count = XHCI_CONTROL_TIMEOUT_DEVIDE;
    unsigned int timeout_sleep = nTimeOut / XHCI_CONTROL_TIMEOUT_DEVIDE;
#endif

    switch(eBit) {
    // Check FW Download enable BIT
    case XHCI_CONTROLREGISTER_BIT_FW_DOWNLOAD_ENABLE:
        mask = PCICNF0F4_FWDOWNLOADENABLE;
        bit  = PCICNF0F4_FWDOWNLOADENABLE_B;
        break;
    // Check FW Download lock BIT
    case XHCI_CONTROLREGISTER_BIT_FW_DOWNLOAD_LOCK:
        mask = PCICNF0F4_FWDOWNLOADLOCK;
        bit  = PCICNF0F4_FWDOWNLOADLOCK_B;
        break;
    // Check Set DATA0 BIT
    case XHCI_CONTROLREGISTER_BIT_SET_DATA0:
        mask = PCICNF0F4_SETDATA0;
        bit  = PCICNF0F4_SETDATA0_B;
        break;
    // Check Set DATA1 BIT
    case XHCI_CONTROLREGISTER_BIT_SET_DATA1:
        mask = PCICNF0F4_SETDATA1;
        bit  = PCICNF0F4_SETDATA1_B;
        break;
    case XHCI_CONTROLREGISTER_RESULT:
        mask = PCICNF0F4_RESULT;
        bit  = PCICNF0F4_RESULT_B;
        break;
    default:
        result = XHCI_FWDOWNLOADER_ERROR;
        break;
    }

#ifdef XHCI_CONTROL_TIMEOUT_NOSLEEP
    start = current_kernel_time();
    while(1) {
        unsigned int read_data;
        struct timespec sub_time;

        // Check specified bit to change
        if(cfgprd(pDev, PCICNF0F4, &read_data) == XHCI_FWDOWNLOADER_SUCCESS) {
            if(((read_data & mask) >> bit) == (unsigned int)eValue) {
                result = XHCI_FWDOWNLOADER_SUCCESS;
                  break;
            }
        }
        now = current_kernel_time();
        sub_time = timespec_sub(now, start);
        if(sub_time.tv_sec > 0) {
            // To fail if the difference is more than one second.
            break;
        }
    }
#else
    for(; (timeout_count > 0)&&(result != XHCI_FWDOWNLOADER_SUCCESS); timeout_count--) {
        // Check the elapsed time specified by the timeout, an error exit if the elapse
        unsigned int read_data;

        if(cfgprd(pDev, PCICNF0F4, &read_data) == XHCI_FWDOWNLOADER_SUCCESS) {
            if(((read_data & mask) >> bit) == (unsigned int)eValue) {
                result = XHCI_FWDOWNLOADER_SUCCESS;
                  break;
            }
        }
        msleep(timeout_sleep);
    }
#endif

    return result;
}

//----------------------------------------------------------
static XHCI_FWDOWNLOADER XHCI_SetControl (
    struct pci_dev                  *pDev,      // Target Device(720201/720202)
    XHCI_CONTROLREGISTER_BIT        eBit        // Bit name for set
) {
    unsigned int read_data;
    XHCI_FWDOWNLOADER result;
    result = cfgprd(pDev, PCICNF0F4, &read_data);

    // To set a specified bit in Control Register
    if(result == XHCI_FWDOWNLOADER_SUCCESS) {
        switch(eBit) {
        case XHCI_CONTROLREGISTER_BIT_FW_DOWNLOAD_ENABLE:
#ifdef XHCI_DEBUG_FLAG_ENABLE
            XHCI_DebugPrint("Set FW Download enable");
#endif
            result = cfgpw(pDev, PCICNF0F4, read_data | PCICNF0F4_FWDOWNLOADENABLE);
            break;
        case XHCI_CONTROLREGISTER_BIT_FW_DOWNLOAD_LOCK:
#ifdef XHCI_DEBUG_FLAG_ENABLE
            XHCI_DebugPrint("Set FW Download lock");
#endif
            result = cfgpw(pDev, PCICNF0F4, read_data | PCICNF0F4_FWDOWNLOADLOCK);
            break;
        case XHCI_CONTROLREGISTER_BIT_SET_DATA0:
#ifdef XHCI_DEBUG_FLAG_ENABLE
            XHCI_DebugPrint("Set SET DATA 0");
#endif
            // CAUTION!!
            // When you set the SetData0 is always SetData1 must be written to 0.
//            result = cfgpw(pDev, PCICNF0F4, read_data | PCICNF0F4_SETDATA0);
            result = cfgpw(pDev, PCICNF0F4, (read_data & ~PCICNF0F4_SETDATA1) | PCICNF0F4_SETDATA0);
            break;
        case XHCI_CONTROLREGISTER_BIT_SET_DATA1:
#ifdef XHCI_DEBUG_FLAG_ENABLE
            XHCI_DebugPrint("Set SET DATA 1");
#endif
            // CAUTION!!
            // When you set the SetData1 is always SetData0 must be written to 0.
//            result = cfgpw(pDev, PCICNF0F4, read_data | PCICNF0F4_SETDATA1);
            result = cfgpw(pDev, PCICNF0F4, (read_data & ~PCICNF0F4_SETDATA0) | PCICNF0F4_SETDATA1);
            break;
        default:
            break;
        }
    }

    return result;
}

//----------------------------------------------------------
static XHCI_FWDOWNLOADER XHCI_ClearControl (
    struct pci_dev                  *pDev,      // Target Device(720201/720202)
    XHCI_CONTROLREGISTER_BIT        eBit        // Bit name for set
) {
    unsigned int read_data;
    XHCI_FWDOWNLOADER result;
    result = cfgprd(pDev, PCICNF0F4, &read_data);

    // Clear the bits specified in the Control register.
    if(result == XHCI_FWDOWNLOADER_SUCCESS) {
        switch(eBit) {
        case XHCI_CONTROLREGISTER_BIT_FW_DOWNLOAD_ENABLE:
#ifdef XHCI_DEBUG_FLAG_ENABLE
            XHCI_DebugPrint("Clear FW Download enable");
#endif
            result = cfgpw(pDev, PCICNF0F4, read_data & ~PCICNF0F4_FWDOWNLOADENABLE);
            break;
        case XHCI_CONTROLREGISTER_BIT_FW_DOWNLOAD_LOCK:
        case XHCI_CONTROLREGISTER_BIT_SET_DATA0:
        case XHCI_CONTROLREGISTER_BIT_SET_DATA1:
        default:
            break;
        }
    }

    return result;
}

//----------------------------------------------------------
XHCI_FWDOWNLOADER XHCI_FWDownLoad (
    struct pci_dev  *pDev,      // Target Device handler
    unsigned char   *pFWImage,  // Pointer to the image to download FW
    unsigned int   nFWSize     // Size of the image to download FW
) {

typedef enum {
    SET_DATA_PAGE0,
    SET_DATA_PAGE1
} SET_DATA;

    SET_DATA set_data_page = SET_DATA_PAGE0;
    unsigned int read_data;
    int offset;
    unsigned int *image_pointer = (unsigned int *)pFWImage;
    unsigned int fw_dwordsize   = nFWSize / (sizeof(unsigned int) / sizeof(unsigned char));

#ifdef XHCI_DEBUG_FLAG_ENABLE
    XHCI_DebugPrint("[RENESAS FWDownload]In XHCI_FWDownLoad()\n");
#endif
    DebugPrintFunctionName();

    if((nFWSize % (sizeof(unsigned int) / sizeof(unsigned char))) != 0) {
        fw_dwordsize++;
    }

    //Check download status
   if(cfgprd (pDev, PCICNF0F4, &read_data) != XHCI_FWDOWNLOADER_SUCCESS) {
        XHCI_DebugPrint("PCICNF0F4 register read error");
        return XHCI_FWDOWNLOADER_ERROR;
    }
    else {
        XHCI_DebugPrint("PCICNF0F4 register = %08x)", read_data);
    }

    if ((read_data & PCICNF0F4_FWDOWNLOADENABLE) != 0) {
        //not ready for a download
        XHCI_DebugPrint("FW Download enable bit was already set(read data = %08x)", read_data);
        return XHCI_FWDOWNLOADER_ERROR;
    }
    if ((read_data & PCICNF0F4_FWDOWNLOADLOCK) != 0) {
        //not ready for a download
        XHCI_DebugPrint("FW Download lock bit was set(read data = %08x)", read_data);
        return XHCI_FWDOWNLOADER_SUCCESS;
    }
    if (((read_data & PCICNF0F4_RESULT) >> PCICNF0F4_RESULT_B) == XHCI_FWDOWNLOAD_STATUS_SUCCESS) {
        // FW has already been downloaded
        XHCI_DebugPrint("Result code is SUCCESS(FW already downloaded)(read data = %08x)", read_data);
        return XHCI_FWDOWNLOADER_SUCCESS;
    }

    //Start fw download
    //a: Set FW Download Enable.
    if(XHCI_SetControl(
        pDev,
        XHCI_CONTROLREGISTER_BIT_FW_DOWNLOAD_ENABLE
        ) == XHCI_FWDOWNLOADER_ERROR) {
     XHCI_DebugPrint("Set FW Download Enable is timeout");
        return XHCI_FWDOWNLOADER_ERROR;
    }
    // expecting result code = 0
    if(XHCI_CheckControlWithTimeOut(
        pDev,
        XHCI_CONTROLREGISTER_RESULT,
        (XHCI_FWDOWNLOADCONTROLBIT_VALUE)XHCI_FWDOWNLOAD_STATUS_INVALID,
        XHCI_CONTROL_TIMEOUT_1000MS
        ) == XHCI_FWDOWNLOADER_ERROR) {
        // timed out waiting for result=0 initially
     XHCI_DebugPrint("Wait result = 0 initially is timeout");
        return XHCI_FWDOWNLOADER_ERROR;
    }


    // Write all the dwords, one pair at a time
    for (offset = 0; offset < fw_dwordsize; offset++) {

        switch(set_data_page) {
        case SET_DATA_PAGE0:
            //b: Read "Set Data0" and confirm it is '0b'.
            if(XHCI_CheckControlWithTimeOut(
                pDev,
                XHCI_CONTROLREGISTER_BIT_SET_DATA0,
                XHCI_FWDOWNLOADCONTROLBIT_DISABLE,
                XHCI_CONTROL_TIMEOUT_1000MS
                ) == XHCI_FWDOWNLOADER_ERROR) {
             // timeout waiting for Dat0 flag to become 0
          XHCI_DebugPrint("Confirm SET DATA0 is to be 0 is timeout(ERROR at writeto offset %04d)", offset);
                return XHCI_FWDOWNLOADER_ERROR;
            }
            //c: Write fw data to "Data0".
            cfgpw (pDev, PCICNF0F8, image_pointer[offset]);
            XHCI_DebugPrint("Set data 0 : offset = %04x, data = %08x", offset, image_pointer[offset]);
            //d: Set "Set Data0".
            if(XHCI_SetControl(
                pDev,
                XHCI_CONTROLREGISTER_BIT_SET_DATA0
                ) == XHCI_FWDOWNLOADER_ERROR) {
                return XHCI_FWDOWNLOADER_ERROR;
            }

#ifdef SINGLE_PAGE
            break;
#else
            set_data_page = SET_DATA_PAGE1;
            break;
        case SET_DATA_PAGE1:
            //e: Read "Set Data1" and confirm it is '0b'.
            if(XHCI_CheckControlWithTimeOut(
                pDev,
                XHCI_CONTROLREGISTER_BIT_SET_DATA1,
                XHCI_FWDOWNLOADCONTROLBIT_DISABLE,
                XHCI_CONTROL_TIMEOUT_1000MS
                ) == XHCI_FWDOWNLOADER_ERROR) {
                // timeout waiting for Dat0:1 flags to become 0
          XHCI_DebugPrint("Confirm SET DATA1 is to be 0 is timeout(ERROR at writeto offset %04d)", offset);
                return XHCI_FWDOWNLOADER_ERROR;
            }

            //f: Write fw data to "Data1".
            cfgpw (pDev, PCICNF0FC, image_pointer[offset]);
            XHCI_DebugPrint("Set data 1 : offset = %04x, data = %08x", offset, image_pointer[offset]);
            //g: Set "Set Data1".
            if(XHCI_SetControl(
                pDev,
                XHCI_CONTROLREGISTER_BIT_SET_DATA1
                ) == XHCI_FWDOWNLOADER_ERROR) {
                return XHCI_FWDOWNLOADER_ERROR;
            }

            set_data_page = SET_DATA_PAGE0;
            break;
#endif
        default:
            break;

        }
    }

    // Wait for DAT0:1 flags to be '00' again
    if(XHCI_CheckControlWithTimeOut(
        pDev,
        XHCI_CONTROLREGISTER_BIT_SET_DATA0,
        XHCI_FWDOWNLOADCONTROLBIT_DISABLE,
        XHCI_CONTROL_TIMEOUT_1000MS
        ) == XHCI_FWDOWNLOADER_ERROR) {
     // timeout waiting for Dat0 flag to become 0
        XHCI_DebugPrint("Confirm SET DATA0 is to be 0(Finally) is timeout");
        return XHCI_FWDOWNLOADER_ERROR;
    }
    if(XHCI_CheckControlWithTimeOut(
        pDev,
        XHCI_CONTROLREGISTER_BIT_SET_DATA1,
        XHCI_FWDOWNLOADCONTROLBIT_DISABLE,
        XHCI_CONTROL_TIMEOUT_1000MS
        ) == XHCI_FWDOWNLOADER_ERROR) {
     // timeout waiting for Dat1 flag to become 0
        XHCI_DebugPrint("Confirm SET DATA1 is to be 0(Finally) is timeout");
        return XHCI_FWDOWNLOADER_ERROR;
    }

    //Stop fw download
    if(XHCI_ClearControl(
        pDev,
        XHCI_CONTROLREGISTER_BIT_FW_DOWNLOAD_ENABLE
        ) == XHCI_FWDOWNLOADER_ERROR) {
        return XHCI_FWDOWNLOADER_ERROR;
    }

    //Check the status of fw download
    if(XHCI_CheckControlWithTimeOut(
        pDev,
        XHCI_CONTROLREGISTER_RESULT,
        (XHCI_FWDOWNLOADCONTROLBIT_VALUE)XHCI_FWDOWNLOAD_STATUS_SUCCESS,
        XHCI_CONTROL_TIMEOUT_1000MS
        ) == XHCI_FWDOWNLOADER_ERROR) {
     // timeout waiting for Dat0 flag to become 0
        XHCI_DebugPrint("clear FW Download enable is timeout");
        return XHCI_FWDOWNLOADER_ERROR;
    }

#if 1
    //h: Set FW Download Lock
    if(XHCI_SetControl(
        pDev,
        XHCI_CONTROLREGISTER_BIT_FW_DOWNLOAD_LOCK
        ) == XHCI_FWDOWNLOADER_ERROR) {
        return XHCI_FWDOWNLOADER_ERROR;
    }
#endif

    return XHCI_FWDOWNLOADER_SUCCESS;
}


//----------------------------------------------------------
XHCI_FWDOWNLOADER XHCI_FWDownLoadCheck(
    struct usb_hcd *hcd     // Target Device handler
) {

    XHCI_FWDOWNLOADER result = XHCI_FWDOWNLOADER_ERROR;
    struct xhci_hcd     *xhci = hcd_to_xhci(hcd);
    struct pci_dev      *pDev = to_pci_dev(hcd->self.controller);

    DebugPrintFunctionName();

    XHCI_DebugPrint("VenderID=%04x, DeviceID=%04x, Revision=%04x", pDev->vendor, pDev->device, pDev->revision);

    // Check the DeviceID, 720201/720202 to confirm that
    if(((pDev->vendor == XHCI_VENDOR_ID_720201)&&(pDev->device == XHCI_DEVICE_ID_720201))
     ||((pDev->vendor == XHCI_VENDOR_ID_720202)&&(pDev->device == XHCI_DEVICE_ID_720202))) {
        // download the FW, if device was 720201/720202
        char *firmwarename = NULL;
        int loopcounter;
//        xhci->firmware_pointer = NULL;

        XHCI_DebugPrint("Device is 720201/202");

		// Check the revision of the device to determine the FW download
		if(pDev->revision == XHCI_DEVICE_REV_ES20) {
            // In the case of 720201/202 revision 2, fw to download
            XHCI_DebugPrint("--Revision 2");
            firmwarename = XHCI_FWFILENAME_720201_202ES20;
        }
        else if(pDev->revision == XHCI_DEVICE_REV_CS) {
            // In the case of 720201 revision 3, to download
            XHCI_DebugPrint("--Revision 3");
            firmwarename = XHCI_FWFILENAME_720201_202ES20;
		}
        else {
            // I don't know that revision is an error
            XHCI_DebugPrint("--Revision ???");
        }

        // Start Download
        for(loopcounter = 0; (loopcounter < 10)&&(xhci->firmware_pointer == NULL); loopcounter++) {
            if(request_firmware((const struct firmware **)&xhci->firmware_pointer, firmwarename, &pDev->bus->dev) == 0) {
//            if(xhci->firmware_pointer != NULL) {
                XHCI_DebugPrint("Firmware load OK");
                result = XHCI_FWDownLoad(pDev, (unsigned char *)xhci->firmware_pointer->data, xhci->firmware_pointer->size);

                // To clean up after the download
#ifndef CONFIG_PM
                release_firmware(xhci->firmware_pointer);
                xhci->firmware_pointer = NULL;
                break;
#endif
             }
            else {
                // If unable to load FW file, succeeds to treatment anyway.
                // (because if it already loaded FW)
                XHCI_DebugPrint("Firmware load ng");
                result = XHCI_FWDOWNLOADER_SUCCESS;
            }
        }
    }
    else {
        // When other devices(720200/720200A/or other) do nothing
        XHCI_DebugPrint("Device is not 720201/202");
        result = XHCI_FWDOWNLOADER_SUCCESS;
    }
    XHCI_DebugPrint("Result=%08x\n", result);

    return result;
}

#ifdef CONFIG_PM
//----------------------------------------------------------
XHCI_FWDOWNLOADER XHCI_FWReLoad(
    struct usb_hcd *hcd     // Target Device handler
) {

    XHCI_FWDOWNLOADER result = XHCI_FWDOWNLOADER_ERROR;
    struct xhci_hcd     *xhci = hcd_to_xhci(hcd);
    struct pci_dev      *pDev = to_pci_dev(hcd->self.controller);

    DebugPrintFunctionName();

    XHCI_DebugPrint("VenderID=%04x, DeviceID=%04x, Revision=%04x", pDev->vendor, pDev->device, pDev->revision);

    // Check the DeviceID, 720201/720202 to confirm that
    if(((pDev->vendor == XHCI_VENDOR_ID_720201)&&(pDev->device == XHCI_DEVICE_ID_720201))
     ||((pDev->vendor == XHCI_VENDOR_ID_720202)&&(pDev->device == XHCI_DEVICE_ID_720202))) {
        // download the FW, if device was 720201/720202

        XHCI_DebugPrint("Device is 720201/202");

        // Start Download
        if(xhci->firmware_pointer != NULL) {
            result = XHCI_FWDownLoad(pDev, (unsigned char *)xhci->firmware_pointer->data, xhci->firmware_pointer->size);
        }
    }
    else {
        // When other devices(720200/720200A/or other) do nothing
        XHCI_DebugPrint("Device is not 720201/202");
        result = XHCI_FWDOWNLOADER_SUCCESS;
    }
    XHCI_DebugPrint("Result=%08x\n", result);
    return result;
}
#endif

//----------------------------------------------------------
XHCI_FWDOWNLOADER XHCI_FWUnLoad(
    struct usb_hcd *hcd     // Target Device handler
) {

    XHCI_FWDOWNLOADER result = XHCI_FWDOWNLOADER_ERROR;
    struct xhci_hcd     *xhci = hcd_to_xhci(hcd);
    struct pci_dev      *pDev = to_pci_dev(hcd->self.controller);

    DebugPrintFunctionName();

    XHCI_DebugPrint("VenderID=%04x, DeviceID=%04x, Revision=%04x", pDev->vendor, pDev->device, pDev->revision);

    // Check the DeviceID, 720201/720202 to confirm that
    if(((pDev->vendor == XHCI_VENDOR_ID_720201)&&(pDev->device == XHCI_DEVICE_ID_720201))
     ||((pDev->vendor == XHCI_VENDOR_ID_720202)&&(pDev->device == XHCI_DEVICE_ID_720202))) {

        XHCI_DebugPrint("Device is 720201/202");

        // To clean the download
#ifdef CONFIG_PM
        if(xhci->firmware_pointer != NULL) {
            release_firmware(xhci->firmware_pointer);
            xhci->firmware_pointer = NULL;
        }
#endif
        result = XHCI_FWDOWNLOADER_SUCCESS;
    }
    else {
        // When other devices(720200/720200A/or other) do nothing
        XHCI_DebugPrint("Device is not 720201/202");
        result = XHCI_FWDOWNLOADER_SUCCESS;
    }
    XHCI_DebugPrint("Result=%08x\n", result);
    return result;
}

