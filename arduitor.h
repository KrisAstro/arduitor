#ifndef ARDUITOR_H
#define ARDUITOR_H

#include "indirotator.h"
#include "config.h"

#include <stdint.h>


class Arduitor : public INDI::Rotator
{
	public:
		Arduitor();
		virtual ~Arduitor() = default;

		
		const char * getDefaultName();
		virtual bool initProperties();
		virtual bool updateProperties();

		//virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);
		//virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);

	protected:
		// Rotator Overrides
		virtual IPState HomeRotator();
		virtual IPState MoveRotator(double angle) override;
		virtual bool ReverseRotator(bool enabled) override;
		virtual bool Handshake();
		virtual bool AbortRotator();
		virtual bool SyncRotator(double angle);
		
		// Misc.
		virtual void TimerHit();

	private:
		// Check if connection is OK
		bool Ack();
		bool sendCommand(const char * cmd, char * res);
		bool isMoving();
		bool readPosition();
		

		IText FirmwareT[1] {};
		ITextVectorProperty FirmwareTP;
		
		const uint8_t ML_RES = 32;
		const uint8_t ML_DEL = '#';
		const uint8_t ML_TIMEOUT = 3;
		
		bool isAborted = false;
};

#endif
