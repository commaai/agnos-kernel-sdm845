Gobi3000 Serial driver 2011-07-29-1026

This readme covers important information concerning 
the Gobi Serial driver.

Table of Contents

1. What's new in this release
2. Known issues
3. Known platform issues


-------------------------------------------------------------------------------

1. WHAT'S NEW

This Release (Gobi3000 Serial driver 2011-07-29-1026)
a. Signal the device to leave low power mode on enumeration
b. Change to new date-based versioning scheme

Prior Release (Gobi3000 Serial driver 1.0.30) 05/18/2011
a. Fix typo affecting 2.6.30 kernels

Prior Release (Gobi3000 Serial driver 1.0.20) 01/05/2011
a. Add support for QDL port on interface 0 (enumeration changed in 
   firmware 1575 and later).

Prior Release (Gobi3000 Serial driver 1.0.10) 09/17/2010
a. Initial release

-------------------------------------------------------------------------------

2. KNOWN ISSUES

No known issues.

-------------------------------------------------------------------------------

3. KNOWN PLATFORM ISSUES

a. If a user attempts to obtain a Simple IP address the device may become
   unresponsive to AT commands and will need to be restarted.  This may be
   a result of a problem in one or more of the following open source products:
      1. pppd daemon
      2. ppp protocol stack
      3. USB driver
   In any case, this is not an issue that can be addressed by any software
   provided by Qualcomm.
b. There is a bug in the open source ehci-hcd driver on kernel 2.6.32
   and newer caused by commit 403dbd36739e344d2d25f56ebbe342248487bd48.  
   This change in the ehci-hcd driver made it so Intel USB hubs are trusted
   to provide hardware interrupts, however some devices do not do so 
   consistently.  This has been observed as a 2 second delay or no URB 
   callback at all, resulting in QDL timeouts or a hang in the n_tty_write()
   function.  This can be corrected by reverting the original commit or 
   specifying need_io_watchdog = 1 for this USB hub.

-------------------------------------------------------------------------------



