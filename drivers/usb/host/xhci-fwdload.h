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
//      2011-11-17 rev1.2     Add to download FW return from suspend.
//
///////////////////////////////////////////////////////////////////////////////


#include "linux/pci.h"


typedef enum {
    XHCI_FWDOWNLOADER_SUCCESS,
    XHCI_FWDOWNLOADER_ERROR,

} XHCI_FWDOWNLOADER;


XHCI_FWDOWNLOADER XHCI_FWDownLoad (
    struct pci_dev  *pDev,      // Target Device handler
    unsigned char   *pFWImage,  // Pointer to the image to download FW
    unsigned int   nFWSize     // Size of the image to download FW
);

XHCI_FWDOWNLOADER XHCI_FWDownLoadCheck(
    struct usb_hcd *hcd     // Target Device handler
);

#ifdef CONFIG_PM
XHCI_FWDOWNLOADER XHCI_FWReLoad(
    struct usb_hcd *hcd     // Target Device handler
);
#endif

XHCI_FWDOWNLOADER XHCI_FWUnLoad(
    struct usb_hcd *hcd     // Target Device handler
);


#define XHCI_FWDOWNLOAD(xcd) if(XHCI_FWDownLoadCheck(xcd) != XHCI_FWDOWNLOADER_SUCCESS) return -ENODEV
#ifdef CONFIG_PM
#define XHCI_FWRELOAD(xcd)   if(XHCI_FWReLoad(xcd) != XHCI_FWDOWNLOADER_SUCCESS) return -ENODEV
#endif
#define XHCI_FWUNLOAD(xcd)   XHCI_FWUnLoad(xcd)

