/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "AP_Ranging.h"
#if AP_RANGING_ENABLED

const AP_Param::GroupInfo AP_Ranging::AnchorParams::var_info[] = {
    // @Param: EN
    // @DisplayName: Surveyed anchor enabled
    // @Description: Enable this fixed anchor ID. Zero coordinates are valid and do not disable the anchor.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EN", 0, AP_Ranging::AnchorParams, enable, 0),
    // @Param: POS
    // @DisplayName: Surveyed anchor position
    // @Description: Antenna position relative to the beacon origin, north east down. Positive Z is below the origin.
    // @Units: m
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("POS", 1, AP_Ranging::AnchorParams, position, 0),
    AP_GROUPEND
};

#endif
