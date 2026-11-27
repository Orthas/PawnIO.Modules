//  PawnIO Modules - Modules for various hardware to be used with PawnIO.
//  Copyright (C) 2026  Orthas
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
//  SPDX-License-Identifier: LGPL-2.1-or-later

//  HP notebook EC "touchpad activity" notification.
//
//  Several HP consumer notebooks switch the keyboard backlight off after a
//  fixed ~30 s of inactivity with no firmware option to change it. The EC
//  restarts that timer whenever it receives a touchpad activity notification.
//  HP's Synaptics driver (SynTP.sys) sends that notification as EC command
//  0x97 followed by one data byte: 0x04 when a finger lands on the pad and
//  0x05 when it lifts. With SynTP's KBCNotifyConfig bit 0x80 set, the bytes go
//  to a second EC command/data pair at 0x6C (status/command) and 0x68 (data)
//  instead of the 8042 ports 0x64/0x60.
//
//  Reference: SynTP.sys 19.5.35.54 (24 Feb 2020), the routines gated by
//  KBCNotifyConfig bit 0x20 (send 0x97 + 0x?4/0x?5) and the bit-0x80 writers
//  (port pair 0x6C/0x68). With SynTP's "SendClearState" setting (default 1)
//  the writer waits only for IBF clear before the command, and after the data
//  byte has been accepted (or on any timeout) writes 0xFF to 0x6C to clear
//  the EC's transaction state. Without that 0xFF, status bit 7 was observed
//  to stay set and the notification had no effect on hardware. Data byte:
//  low nibble 4 (finger down) / 5 (finger up); SynTP's HID-path class adds
//  flag bits 0x10 / 0x40 in the high nibble. On the supported board
//  KBCNotifyConfig is 0xA0. The ports are not claimed by any PnP device and
//  no DSDT OperationRegion covers them.
//
//  Scope is deliberately minimal: the module can only write command 0x97, a
//  data byte from {0x04,0x05,0x14,0x15,0x44,0x45,0x54,0x55} and the 0xFF
//  clear-state byte to this pair, and only read the status port. It cannot be
//  used as a general port I/O or EC RAM interface.
//
//  The same notification is sent by SynTP on every touch, so this module does
//  nothing the platform does not already do in normal use. Callers should set
//  SynTP's KBCNotifyConfig to 0 while using it so two writers do not
//  interleave command and data bytes.

#include <pawnio.inc>
#include <registry.inc>

#define EC_STATUS_CMD_PORT  0x6C
#define EC_DATA_PORT        0x68

#define EC_STS_IBF          0x02    // input buffer full
#define EC_PREWAIT_LEGACY   0x82    // SynTP's pre-command mask when SendClearState=0

#define NOTIFY_CMD          0x97
#define EC_CLEAR_STATE      0xFF    // SynTP SendClearState terminator

#define EC_POLL_US          50
#define EC_POLL_COUNT       4000    // 4000 x 50 us = 200 ms per handshake step

/// Compare two NUL-terminated codepoint strings.
///
/// @param str1 Unpacked string (one codepoint per cell)
/// @param str2 Unpacked string (one codepoint per cell)
/// @return 0 if equal, nonzero otherwise
stock str_eq(const str1[], const str2[]) {
    new i = 0;
    while (str1[i] != 0) {
        new c = str2[i];
        if (c == 0)
            return 1;
        if (str1[i] != c)
            return 1;
        i++;
    }
    return str2[i] != 0 ? 1 : 0;
}

// Boards on which the notification channel and bytes have been verified
// (SynTP disassembly + hardware test). Add boards only after verifying
// KBCNotifyConfig has bit 0x80 set on them.
bool:is_supported_board(const board[]) {
    // HP ENVY x360 Convertible 15-ee0xxx (AMD Ryzen 4000)
    return str_eq(board, ''876F'') == 0;
}

bool:is_supported_machine() {
    if (!reg_resolve_imports())
        return false;

    new bios_path[] = ''\\Registry\\Machine\\HARDWARE\\DESCRIPTION\\System\\BIOS'';

    new manufacturer[64];
    new mfg_len = 0;
    if (reg_query_sz(bios_path, ''BaseBoardManufacturer'', manufacturer, sizeof manufacturer, mfg_len) != STATUS_SUCCESS)
        return false;
    if (str_eq(manufacturer, ''HP'') != 0)
        return false;

    new board[32];
    new board_len = 0;
    if (reg_query_sz(bios_path, ''BaseBoardProduct'', board, sizeof board, board_len) != STATUS_SUCCESS)
        return false;

    return is_supported_board(board);
}

// Wait until (status & mask) == 0. Bounded to EC_POLL_COUNT polls.
NTSTATUS:ec_wait_clear(mask) {
    for (new i = 0; i < EC_POLL_COUNT; i++) {
        if ((io_in_byte(EC_STATUS_CMD_PORT) & mask) == 0)
            return STATUS_SUCCESS;
        microsleep(EC_POLL_US);
    }
    return STATUS_IO_TIMEOUT;
}

// Allowed data bytes: low nibble 4 (finger down) or 5 (finger up), high
// nibble 0 or any combination of SynTP's 0x10 / 0x40 flag bits.
bool:is_allowed_code(code) {
    new lo = code & 0x0F;
    new hi = code & 0xF0;
    if (lo != 0x04 && lo != 0x05)
        return false;
    return hi == 0x00 || hi == 0x10 || hi == 0x40 || hi == 0x50;
}

/// Send a touchpad activity notification to the EC (command 0x97).
///
/// This is the same notification HP's SynTP driver sends on every touch. The
/// EC restarts its keyboard backlight inactivity timer when it receives it.
///
/// Mode 0 (SynTP default, SendClearState=1): wait for IBF clear, write 0x97,
/// wait, write the data byte, wait, then write 0xFF to 0x6C. 0xFF is also
/// written if any wait times out, exactly as SynTP does.
/// Mode 1 (SendClearState=0): wait for (status & 0x82) == 0 before the
/// command and never write 0xFF.
///
/// @param in [0] = Data byte: 0x04/0x05 (finger down/up), optionally with
///                 flag bits 0x10 and/or 0x40 set,
///               [1] = Mode: 0 = SendClearState (default), 1 = legacy
/// @param in_size Must be 2
/// @param out [0] = EC status byte (port 0x6C) after the transaction
/// @param out_size Must be 1
/// @return An NTSTATUS
/// @retval STATUS_INVALID_PARAMETER The data byte or mode is not one of the allowed values
/// @retval STATUS_IO_TIMEOUT The EC did not accept the command or data byte within 200 ms
DEFINE_IOCTL_SIZED(ioctl_touch_notify, 2, 1) {
    new code = in[0];
    new mode = in[1];

    if (!is_allowed_code(code))
        return STATUS_INVALID_PARAMETER;
    if (mode != 0 && mode != 1)
        return STATUS_INVALID_PARAMETER;

    new bool:clear_state = mode == 0;
    new prewait = clear_state ? EC_STS_IBF : EC_PREWAIT_LEGACY;

    new NTSTATUS:status = ec_wait_clear(prewait);
    if (NT_SUCCESS(status)) {
        io_out_byte(EC_STATUS_CMD_PORT, NOTIFY_CMD);
        status = ec_wait_clear(EC_STS_IBF);
        if (NT_SUCCESS(status)) {
            io_out_byte(EC_DATA_PORT, code);
            status = ec_wait_clear(EC_STS_IBF);
        }
    }

    if (clear_state)
        io_out_byte(EC_STATUS_CMD_PORT, EC_CLEAR_STATE);

    out[0] = io_in_byte(EC_STATUS_CMD_PORT);
    return status;
}

/// Read the EC status byte of the notification channel (port 0x6C).
///
/// @param in Unused
/// @param in_size Unused
/// @param out [0] = Status byte
/// @param out_size Must be 1
/// @return An NTSTATUS
DEFINE_IOCTL_SIZED(ioctl_status, 0, 1) {
    out[0] = io_in_byte(EC_STATUS_CMD_PORT);
    return STATUS_SUCCESS;
}

NTSTATUS:main() {
    if (get_arch() != ARCH_X64)
        return STATUS_NOT_SUPPORTED;

    if (!is_supported_machine())
        return STATUS_NOT_SUPPORTED;

    // An undecoded LPC port reads 0xFF; a live EC status byte never has
    // every bit set.
    if (io_in_byte(EC_STATUS_CMD_PORT) == 0xFF)
        return STATUS_NOT_SUPPORTED;

    return STATUS_SUCCESS;
}
