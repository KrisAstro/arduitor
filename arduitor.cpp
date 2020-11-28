/*******************************************************************************
  Copyright(c) 2020 Christian Picard. All rights reserved.

  Arduitor rotator

  This program is free software; you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; see the file COPYING.LIB.  If not, write to
  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
  Boston, MA 02110-1301, USA.

  The full GNU General Public License is included in this distribution in the
  file called LICENSE.
*******************************************************************************/



#include <cmath>
#include <memory>
#include <regex>
#include <termios.h>
#include <cstring>
#include <sys/ioctl.h>
#include <chrono>
#include <math.h>
#include <iomanip>
#include <chrono>
#include <thread>

#include "arduitor.h"
#include "connectionplugins/connectionserial.h"
#include "indicom.h"


static std::unique_ptr<Arduitor> arduitor(new Arduitor());


void ISGetProperties(const char *dev)
{
    arduitor->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    arduitor->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(	const char *dev, const char *name, char *texts[], char *names[], int n)
{
    arduitor->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    arduitor->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[],
                char *formats[], char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}

void ISSnoopDevice (XMLEle *root)
{
    arduitor->ISSnoopDevice(root);
}

//////////////////////////////////////////////////////////////////////
/// 
//////////////////////////////////////////////////////////////////////
Arduitor::Arduitor()
{
    RI::SetCapability(ROTATOR_CAN_HOME | ROTATOR_CAN_REVERSE | ROTATOR_CAN_ABORT | ROTATOR_CAN_SYNC);

    setRotatorConnection(CONNECTION_SERIAL);
}

//////////////////////////////////////////////////////////////////////
/// 
//////////////////////////////////////////////////////////////////////
bool Arduitor::initProperties()
{
    INDI::Rotator::initProperties();

    addAuxControls();

    // Firmware version
    IUFillText(&FirmwareT[0], "FIRMWARE_VERSION", "Version", "Unknown");
    IUFillTextVector(&FirmwareTP, FirmwareT, 1, getDeviceName(), "FIRMWARE_VERSION", "Firmware", INFO_TAB, IP_RO, 0, IPS_IDLE);


	serialConnection->setDefaultPort("/dev/rotator");
	serialConnection->setDefaultBaudRate(Connection::Serial::B_19200);

	return true;
}

//////////////////////////////////////////////////////////////////////
/// 
//////////////////////////////////////////////////////////////////////
bool Arduitor::Handshake()
{
    if (Ack())
        return true;

    LOG_INFO("Error retrieving data from Arduitor, please ensure Arduitor controller is powered and the port is correct.");
    return false;
}

const char * Arduitor::getDefaultName()
{
    return "Arduitor";
}

bool Arduitor::updateProperties()
{
    INDI::Rotator::updateProperties();

    if (isConnected())
    {
        defineText(&FirmwareTP) ;
    }
    else
    {
        deleteProperty(FirmwareTP.name) ;
    }

	return true;
}

//////////////////////////////////////////////////////////////////////
/// 
//////////////////////////////////////////////////////////////////////
bool Arduitor::Ack()
{
    char res[ML_RES] = {0};

	for(uint8_t i = 0; i < 3; i++)
	{
		if (sendCommand(":GV#", res))
		{
			LOG_INFO("Arduitor is online. Getting Rotator parameters...");
			char buf[10];
			sprintf(buf, "%c.%c", res[0], res[1]);
			IUSaveText(&FirmwareT[0], buf);
			return true;
		}
		
		//sleep(1);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
		
	LOG_ERROR("Arduitor not reconized.");

	return false;
}

//////////////////////////////////////////////////////////////////////
/// 
//////////////////////////////////////////////////////////////////////
void Arduitor::TimerHit()
{
    if(!isConnected())
	{
		SetTimer(POLLMS);
		return;
	}

	char res[ML_RES] = {0};

	if(sendCommand(":GP#", res))
	{
		uint16_t position = std::stoi(res);
		
		const IPState motionState = isMoving() ? IPS_BUSY : IPS_OK;
		
        // Update Absolute Position property if either position changes, or status changes.
        if (std::abs(position - GotoRotatorN[0].value) > 0.01 || GotoRotatorNP.s != motionState)
        {
            GotoRotatorN[0].value = position;
            GotoRotatorNP.s = motionState;
            IDSetNumber(&GotoRotatorNP, nullptr);
        }
	}

    if(HomeRotatorSP.s == IPS_BUSY)
	{
		if(!isMoving())
		{
            HomeRotatorSP.s = IPS_OK;
            HomeRotatorS[0].s = ISS_OFF;
            IDSetSwitch(&HomeRotatorSP, nullptr);
            
            LOG_INFO("Homing is complete.");
		}
	}
	
	SetTimer(POLLMS);
}

//////////////////////////////////////////////////////////////////////
/// 
//////////////////////////////////////////////////////////////////////
bool Arduitor::isMoving()
{
	char res[ML_RES] = {0};
	
	if( sendCommand(":GI#", res) )
	{
		if(strstr(res, "01#"))
		{
			return true;
		}
	}
	
	return false;
}

//////////////////////////////////////////////////////////////////////
/// Serial commands
//////////////////////////////////////////////////////////////////////
bool Arduitor::sendCommand(const char * cmd, char * res)
{
    int nbytes_written = 0, 
		nbytes_read = 0, 
		rc = -1;

    tcflush(PortFD, TCIOFLUSH);

    LOGF_DEBUG("CMD <%s>", cmd);

    if ((rc = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial write error: %s.", errstr);

        return false;
    }

    if (res == nullptr)
    {
        tcdrain(PortFD);
        return true;
    }

    rc = tty_nread_section(PortFD, res, ML_RES, ML_DEL, ML_TIMEOUT, &nbytes_read);

    if (rc != TTY_OK || nbytes_read == 0)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial read error: %s.", errstr);

        return false;
    }

    LOGF_DEBUG("RES <%s>", res);

    tcflush(PortFD, TCIOFLUSH);

    return true;
}


//////////////////////////////////////////////////////////////////////
/// 
//////////////////////////////////////////////////////////////////////
IPState Arduitor::HomeRotator()
{
	if( ! sendCommand(":PH#", nullptr) )
	{
		LOG_ERROR( "GoHome error");
		return IPS_ALERT;
	}
		
	return IPS_BUSY;
}

//////////////////////////////////////////////////////////////////////
/// 
//////////////////////////////////////////////////////////////////////
IPState Arduitor::MoveRotator(double angle)
{
    char cmd[ML_RES] = {0};
	snprintf(cmd, ML_RES, ":SN%04X#", (uint16_t)angle);

	LOGF_DEBUG("MoveRotator <%s>", cmd);
	
	if(sendCommand(cmd, nullptr))
	{
		if(sendCommand(":FG#", nullptr))
			return IPS_BUSY;
	}	

	return IPS_ALERT;
}

//////////////////////////////////////////////////////////////////////
/// 
//////////////////////////////////////////////////////////////////////
bool Arduitor::AbortRotator()
{
	if( (isAborted = sendCommand(":FQ#", nullptr)))
		return true;
		
	return false;
}
	
//////////////////////////////////////////////////////////////////////
/// 
//////////////////////////////////////////////////////////////////////
bool Arduitor::ReverseRotator(bool enabled)
{
	
	if(sendCommand(enabled ? ":CC#" : ":CW#", nullptr))
		return true;
		
	return false;
}

//////////////////////////////////////////////////////////////////////
/// 
//////////////////////////////////////////////////////////////////////
bool Arduitor::SyncRotator(double angle)
{
    char cmd[ML_RES] = {0};
	snprintf(cmd, ML_RES, ":SP%04X#", (uint16_t)angle);
	
	return sendCommand(cmd, nullptr);
	
}

