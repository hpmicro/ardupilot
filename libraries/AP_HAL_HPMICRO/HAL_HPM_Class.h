/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Namespace.h>
#include <AP_HAL_HPMICRO/HAL_HPM_Namespace.h>

class HAL_HPM : public AP_HAL::HAL
{
public:
    HAL_HPM();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};

typedef HPMicro::CANIface HAL_CANIface;