#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

cmake_minimum_required(VERSION 3.20.0)

# Use the bt-ll-sw-split snippet provided by Nordic in the nRF Connect SDK. This
# snippet selects the Zephyr Bluetooth LE Controller (bt-ll-sw-split) instead of
# the Nordic SoftDevice Controller (SDC). The Nordic SDC supports connectionless
# CTE advertising for AoA, but not for AoD. The Zephyr BLE Controller supports
# connectionless CTE advertising for both AoA and AoD.
if(NOT bt-ll-sw-split IN_LIST SNIPPET)
  set(SNIPPET bt-ll-sw-split ${SNIPPET} CACHE STRING "" FORCE)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(beacon)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
)
# NORDIC SDK APP END
