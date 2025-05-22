/******************************************************************************
 *
 *  Copyright (C) 2006-2023  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT master userspace library.
 *
 *  The IgH EtherCAT master userspace library is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU Lesser General
 *  Public License as published by the Free Software Foundation; version 2.1
 *  of the License.
 *
 *  The IgH EtherCAT master userspace library is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with the IgH EtherCAT master userspace library. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

/** \file
 * Canopen over EtherCAT SoE request functions.
 */

/*****************************************************************************/

#include <stdio.h>
#include <string.h>

#include "ioctl.h"
#include "soe_request.h"
#include "slave_config.h"
#include "master.h"

/*****************************************************************************/

void ec_soe_request_clear(ec_soe_request_t *req)
{
    if (req->data) {
        free(req->data);
        req->data = NULL;
    }
}

/*****************************************************************************
 * Application interface.
 ****************************************************************************/

void ecrt_soe_request_idn(ec_soe_request_t *req, uint8_t drive_no,
        uint16_t idn)
{
    ec_ioctl_soe_request_t data;
    int ret;

    data.config_index = req->config->index;
    data.request_index = req->index;
    data.drive_no = drive_no;
    data.idn = idn;

    ret = ioctl(req->config->master->fd, EC_IOCTL_SOE_REQUEST_IDN, &data);
    if (EC_IOCTL_IS_ERROR(ret)) {
        fprintf(stderr, "Failed to set SoE request IDN: %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
    }
}

/*****************************************************************************/

void ecrt_soe_request_timeout(ec_soe_request_t *req, uint32_t timeout)
{
    ec_ioctl_soe_request_t data;
    int ret;

    data.config_index = req->config->index;
    data.request_index = req->index;
    data.timeout = timeout;

    ret = ioctl(req->config->master->fd, EC_IOCTL_SOE_REQUEST_TIMEOUT, &data);
    if (EC_IOCTL_IS_ERROR(ret)) {
        fprintf(stderr, "Failed to set SoE request timeout: %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
    }
}

/*****************************************************************************/

uint8_t *ecrt_soe_request_data(ec_soe_request_t *req)
{
    return req->data;
}

/*****************************************************************************/

size_t ecrt_soe_request_data_size(const ec_soe_request_t *req)
{
    return req->data_size;
}

/*****************************************************************************/

ec_request_state_t ecrt_soe_request_state(ec_soe_request_t *req)
{
    ec_ioctl_soe_request_t data;
    int ret;

    data.config_index = req->config->index;
    data.request_index = req->index;

    ret = ioctl(req->config->master->fd, EC_IOCTL_SOE_REQUEST_STATE, &data);
    if (EC_IOCTL_IS_ERROR(ret)) {
        fprintf(stderr, "Failed to get SoE request state: %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
        return EC_REQUEST_ERROR;
    }

    if (data.size) { // new data waiting to be copied
        if (req->mem_size < data.size) {
            fprintf(stderr, "Received %zu bytes do not fit info SoE data"
                    " memory (%zu bytes)!\n", data.size, req->mem_size);
            return EC_REQUEST_ERROR;
        }

        data.data = req->data;

        ret = ioctl(req->config->master->fd,
                EC_IOCTL_SOE_REQUEST_DATA, &data);
        if (EC_IOCTL_IS_ERROR(ret)) {
            fprintf(stderr, "Failed to get SoE data: %s\n",
                    strerror(EC_IOCTL_ERRNO(ret)));
            return EC_REQUEST_ERROR;
        }
        req->data_size = data.size;
    }

    return data.state;
}

/*****************************************************************************/

void ecrt_soe_request_read(ec_soe_request_t *req)
{
    ec_ioctl_soe_request_t data;
    int ret;

    data.config_index = req->config->index;
    data.request_index = req->index;

    ret = ioctl(req->config->master->fd, EC_IOCTL_SOE_REQUEST_READ, &data);
    if (EC_IOCTL_IS_ERROR(ret)) {
        fprintf(stderr, "Failed to command an SoE read operation : %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
    }
}

/*****************************************************************************/

void ecrt_soe_request_write(ec_soe_request_t *req)
{
    ec_ioctl_soe_request_t data;
    int ret;

    data.config_index = req->config->index;
    data.request_index = req->index;
    data.data = req->data;
    data.size = req->data_size;

    ret = ioctl(req->config->master->fd, EC_IOCTL_SOE_REQUEST_WRITE, &data);
    if (EC_IOCTL_IS_ERROR(ret)) {
        fprintf(stderr, "Failed to command an SDO write operation : %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
    }
}

/*****************************************************************************/
