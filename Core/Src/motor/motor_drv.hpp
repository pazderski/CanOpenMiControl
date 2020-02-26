#pragma once
#include "mi_control.hpp"
#include "string.h"
#include "math.h"


class MotorDrv
{
	typedef struct
	{
		uint32_t index;
		int32_t pos;
		int32_t vel;
	} __attribute__ ((packed)) ReceiveData1;

	typedef struct
	{
		uint32_t index;
		int16_t current;
		uint16_t status;
		uint32_t res;
	} __attribute__ ((packed)) ReceiveData2;

public:
	Sdo sdo;
	Pdo pdo;
	Nmt nmt;

	uint16_t id;

	enum State {Idle = 0, Configured = 1, Operational = 2, Waiting = 3};

	State state;

	float desiredVel;
	float desiredCurrent;

	int32_t measuredVel;
	int32_t measuredPos;
	int32_t measuredCurrent;
	int32_t actualStatus;

	float position;

	static constexpr float impToRadRatio = 2*M_PI/(2000.0*53.0);

	MotorDrv(CanDrv * canDrv, uint8_t id) : sdo(canDrv, id), pdo(canDrv, id), nmt(canDrv, id)
	{
		this->id = id;
		state = Idle;
		desiredVel = 0;
		SetPdoCmds();

	}

	void SetPdoCmds()
	{
		pdo.cmdTx[Pdo::Pdo1].size = 4;
	}

	void Configure()
	{
		sdo.PushCommand(MiControlCmds::ClearError());
		sdo.PushCommand(MiControlCmds::RestoreParam());
		sdo.PushCommand(MiControlCmds::MotorEnable());

		sdo.StartSequence();
	}


	void SetVelocity()
	{
		pdo.cmdTx[Pdo::Pdo1].data0 = (int32_t) desiredVel;
		pdo.Send(Pdo::Pdo1);
	}

	void SetCurrent()
	{
		pdo.cmdTx[Pdo::Pdo1].data0 = (int32_t) desiredCurrent;
		pdo.Send(Pdo::Pdo1);
	}

	void ReadPosition(volatile CanMsg * msg)
	{
		auto data = (ReceiveData1 *) msg;
		measuredPos = data->pos;
		measuredVel = data->vel;
		position = (float)measuredPos * impToRadRatio;
	}

	void ReadCurrent(volatile CanMsg * msg)
	{
		auto data = (ReceiveData2 *) msg;
		measuredCurrent = data->current;
	}

	void ReadStatus(volatile CanMsg * msg)
	{
		//measuredCurrent = (int32_t) msg->data[1];
	}

};
