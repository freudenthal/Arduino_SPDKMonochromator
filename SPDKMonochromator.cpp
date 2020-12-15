#include "SPDKMonochromator.h"

const uint8_t SPDKMonochromator::CompeletedByte = 24;
const uint32_t SPDKMonochromator::ResetCompleteTime = 1000000;
const uint32_t SPDKMonochromator::CommandReplyTimeMax = 250000;
const uint32_t SPDKMonochromator::TimeToCompleteDefault = 100000;
const uint8_t SPDKMonochromator::RetryCountMax = 8;
const uint32_t SPDKMonochromator::WipeInputEvery = 100000;
const SPDKMonochromator::CommandStruct SPDKMonochromator::CommandLibrary[] =
{
	{CommandType::None,0,CommandParameterType::None,CommandReplyType::None,false,false,0},
	{CommandType::Clear,25,CommandParameterType::None,CommandReplyType::None,true,false,10000000},
	{CommandType::CSRMode,28,CommandParameterType::uInt16,CommandReplyType::None,false,false,0},
	{CommandType::Echo,27,CommandParameterType::None,CommandReplyType::None,false,false,0},
	{CommandType::GratingCal,18,CommandParameterType::uInt24,CommandReplyType::None,true,false,0},
	{CommandType::GratingID,19,CommandParameterType::None,CommandReplyType::GratingID,false,false,0},
	{CommandType::GratingSel,26,CommandParameterType::uInt8,CommandReplyType::None,true,false,10000000},
	{CommandType::Reset,255,CommandParameterType::None,CommandReplyType::None,true,false,0},
	{CommandType::SlitReset,43,CommandParameterType::uInt8,CommandReplyType::None,false,false,10000000},
	{CommandType::Scan,12,CommandParameterType::uInt24,CommandReplyType::None,false,false,10000000},
	{CommandType::ScanUp,9,CommandParameterType::None,CommandReplyType::None,false,true,0},
	{CommandType::ScanDown,3,CommandParameterType::None,CommandReplyType::None,false,true,0},
	{CommandType::Serial,33,CommandParameterType::None,CommandReplyType::SerialID,false,false,0},
	{CommandType::SlewUp,8,CommandParameterType::None,CommandReplyType::None,false,true,0},
	{CommandType::SlewDown,2,CommandParameterType::None,CommandReplyType::None,false,true,0},
	{CommandType::SlitWidthGetAll,30,CommandParameterType::None,CommandReplyType::SlitPositions,false,false,0},
	{CommandType::SlitWidthSetAll,14,CommandParameterType::uInt16,CommandReplyType::None,false,false,0},
	{CommandType::SlitWidthSetInput,31,CommandParameterType::uInt16,CommandReplyType::None,false,false,0},
	{CommandType::SlitWidthSetOutput,32,CommandParameterType::uInt16,CommandReplyType::None,false,false,0},
	{CommandType::SlitCalInput,22,CommandParameterType::uInt16,CommandReplyType::None,true,false,0},
	{CommandType::SlitCalOutput,23,CommandParameterType::uInt16,CommandReplyType::None,true,false,0},
	{CommandType::ScanSpeedGet,21,CommandParameterType::None,CommandReplyType::uInt16,false,false,0},
	{CommandType::ScanSpeedSet,13,CommandParameterType::uInt16,CommandReplyType::None,false,false,0},
	{CommandType::StepDown,1,CommandParameterType::None,CommandReplyType::None,false,false,0},
	{CommandType::StepUp,7,CommandParameterType::None,CommandReplyType::None,false,false,0},
	{CommandType::SelfTest,17,CommandParameterType::None,CommandReplyType::None,true,false,10000000},
	{CommandType::WavelengthGet,29,CommandParameterType::None,CommandReplyType::uInt24,false,false,0},
	{CommandType::WavelengthSet,16,CommandParameterType::uInt24,CommandReplyType::None,false,false,10000000},
	{CommandType::WavelengthZero,52,CommandParameterType::uInt8,CommandReplyType::None,false,false,0},
	{CommandType::ScanSlewStop,24,CommandParameterType::None,CommandReplyType::None,false,false,0},
};
SPDKMonochromator::SPDKMonochromator(HardwareSerial *serial)
{
	SerialPort = serial;
	SerialPort->begin(9600, SERIAL_8N1);
	RecievedCallback = NULL;
	CurrentCommand = NULL;
	CurrentCommandParameter = 0;
	CurrentCommandTimeToComplete = 0;
	ReplyByteCount = 0;
	for (uint8_t Index = 0; Index < 6; ++Index)
	{
		ReplyData[Index] = 0;
	}
	ReplyByteCountMax = 6;
	ClearCommandQueue();
	StatusByte = 0;
	Mode = ModeType::Inactive;
	Busy = false;
	Wavelength = 0.0;
	SlitWidthInput = 0;
	SlitWidthOutput = 0;
	GratingCurrent = 0;
	LastCommandSentTime = 0;
	CommandReplyTime = 0;
	StatusByteReceivedTime = 0;
	ResetTime = 0;
	ResetWaitCount = 0;
	CommandRetryCount = 0;
	for (uint8_t Index = 0; Index < 5; ++Index)
	{
		IDNumber[Index] = 0;
	}
	SlitPositionEnterance = 0;
	SlitPositionExit = 0;
	ScanSpeed = 0;
	GratingsInstalled = 0;
	GratingsCurrent = 0;
	GratingCurrentRuling = 0;
	GratingCurrentBlaze = 0;
}
void SPDKMonochromator::Begin()
{
	ClearCommandQueue();
	Enqueue(CommandType::Echo);
	Enqueue(CommandType::Serial);
	Enqueue(CommandType::GratingID);
	Enqueue(CommandType::SlitWidthGetAll);
	Enqueue(CommandType::WavelengthGet);
	Enqueue(CommandType::ScanSpeedGet);
	Enqueue(CommandType::SlitWidthSetAll,1000);
	Mode = ModeType::Idle;
}
void SPDKMonochromator::SetRecievedCallback(FinishedListener Finished)
{
	RecievedCallback = Finished;
}
void SPDKMonochromator::SendSetSlitWidthAll(uint16_t SlitWidth)
{
	Enqueue(CommandType::SlitWidthSetAll,SlitWidth);
}
void SPDKMonochromator::SendSetSlitWidthInput(uint16_t SlitWidth)
{
	Enqueue(CommandType::SlitWidthSetInput,SlitWidth);
}
void SPDKMonochromator::SendSetSlitWidthOutput(uint16_t SlitWidth)
{
	Enqueue(CommandType::SlitWidthSetOutput,SlitWidth);
}
void SPDKMonochromator::SendSetWavelength(float Wavelength)
{
	uint32_t WavelengthToSend = (uint32_t)(Wavelength * 100);
	Enqueue(CommandType::WavelengthSet,WavelengthToSend);
}
void SPDKMonochromator::SendGetWavelength()
{
	Enqueue(CommandType::WavelengthGet);
}
void SPDKMonochromator::SendSetGrating(uint8_t GratingNumber)
{
	GratingNumber = constrain(GratingNumber,1,3);
	Enqueue(CommandType::GratingSel,GratingNumber);
}
void SPDKMonochromator::SendGetGrating()
{
	Enqueue(CommandType::GratingID);
}
void SPDKMonochromator::SendGetSlitWidth()
{
	Enqueue(CommandType::SlitWidthGetAll);
}
uint8_t* SPDKMonochromator::GetIDNumber()
{
	return IDNumber;
}
float SPDKMonochromator::GetWavelength()
{
	return Wavelength;
}
uint16_t SPDKMonochromator::GetScanSpeed()
{
	return ScanSpeed;
}
uint8_t SPDKMonochromator::GetGratingsInstalled()
{
	return GratingsInstalled;
}
uint8_t SPDKMonochromator::GetGratingCurrent()
{
	return GratingCurrent;
}
uint16_t SPDKMonochromator::GetGratingCurrentRuling()
{
	return GratingCurrentRuling;
}
uint16_t SPDKMonochromator::GetGratingCurrentBlaze()
{
	return GratingCurrentBlaze;
}
uint16_t SPDKMonochromator::GetSlitWidthInput()
{
	return SlitWidthInput;
}
uint16_t SPDKMonochromator::GetSlitWidthOutput()
{
	return SlitWidthOutput;
}
void SPDKMonochromator::Check()
{
	switch (Mode)
	{
		case ModeType::Idle:
			CheckCommandQueue();
			break;
		case ModeType::WaitForCommandReply:
			CheckForCommandReply();
			break;
		case ModeType::WaitForParameterReply:
			CheckForParameterReply();
			break;
		case ModeType::WaitForStatus:
			CheckForStatus();
			break;
		case ModeType::WaitForCompleted:
			CheckForCompleted();
			break;
		case ModeType::WaitToSendEcho:
			WaitToSendEcho();
			break;
		default:
			break;
	}
}
void SPDKMonochromator::CheckCommandQueue()
{
	bool NewCommandPulled = CommandQueuePullToCurrentCommand();
	if (NewCommandPulled)
	{
		if (CurrentCommand != NULL)
		{
			Busy = true;
			UpdateCurrentCommandVariables();
			bool SendStatus = SendCommand(CurrentCommand);
			if (SendStatus)
			{
				Mode = ModeType::WaitForCommandReply;
			}
		}
		else
		{
			Serial.print("<MONOERROR>(Command in queue is null.)\n");
		}
	}
	if ( (micros() - LastWipeTime) > WipeInputEvery )
	{
		LastWipeTime = micros();
		if (SerialPort->available())
		{
			//uint8_t ByteRead = SerialPort->read();
			//Serial.print("J:");
			//Serial.print(ByteRead);
			//Serial.print("\n");
			SerialPort->read();
		}
	}
}
void SPDKMonochromator::Enqueue(CommandType Command)
{
	SPDKMonochromator::Enqueue(Command, 0);
}
void SPDKMonochromator::Enqueue(CommandType Command, uint32_t Parameter)
{
	CommandStruct* CommandPointer = const_cast<CommandStruct*>(&CommandLibrary[static_cast<uint8_t>(Command)]);
	SPDKMonochromator::CommandQueuePut(CommandPointer, Parameter);
}
void SPDKMonochromator::ClearCommandQueue()
{
	for (int Index = 0; Index < SPDKMonochromatorQueueCount; ++Index)
	{
		CommandQueue[Index].Command = NULL;
		CommandQueue[Index].Parameter = 0;
	}
	CommandQueueHead = 0;
	CommandQueueTail = 0;
	CommandQueueFullFlag = false;
}
bool SPDKMonochromator::CommandQueueFull()
{
	return CommandQueueFullFlag;
}
bool SPDKMonochromator::CommandQueueEmpty()
{
	return ( !CommandQueueFullFlag && (CommandQueueHead == CommandQueueTail) );
}
uint8_t SPDKMonochromator::CommandQueueCount()
{
	uint8_t Count = SPDKMonochromatorQueueCount;
	if(!CommandQueueFullFlag)
	{
		if(CommandQueueHead >= CommandQueueTail)
		{
			Count = (CommandQueueHead - CommandQueueTail);
		}
		else
		{
			Count = (SPDKMonochromatorQueueCount + CommandQueueHead - CommandQueueTail);
		}
	}
	return Count;
}
void SPDKMonochromator::CommandQueueAdvance()
{
	if(CommandQueueFullFlag)
	{
		CommandQueueTail = (CommandQueueTail + 1) % SPDKMonochromatorQueueCount;
	}
	CommandQueueHead = (CommandQueueHead + 1) % SPDKMonochromatorQueueCount;
	CommandQueueFullFlag = (CommandQueueHead == CommandQueueTail);
}
void SPDKMonochromator::CommandQueueRetreat()
{
	CommandQueueFullFlag = false;
	CommandQueueTail = (CommandQueueTail + 1) % SPDKMonochromatorQueueCount;
}
void SPDKMonochromator::CommandQueuePut(CommandStruct* CommandPointer, uint32_t Parameter)
{
	CommandQueue[CommandQueueHead].Command = CommandPointer;
	CommandQueue[CommandQueueHead].Parameter = Parameter;
	CommandQueueAdvance();
}
bool SPDKMonochromator::CommandQueuePullToCurrentCommand()
{
	bool Status = false;
	if (!CommandQueueEmpty())
	{
		CurrentCommand = CommandQueue[CommandQueueTail].Command;
		CurrentCommandParameter = CommandQueue[CommandQueueTail].Parameter;
		CommandQueueRetreat();
		Status = true;
	}
	return Status;
}
void SPDKMonochromator::UpdateCurrentCommandVariables()
{
	CurrentCommandTimeToComplete = CurrentCommand->TimeToComplete;
	if (CurrentCommandTimeToComplete == 0)
	{
		CurrentCommandTimeToComplete = TimeToCompleteDefault;
	}
	if (CurrentCommand->ReplyType != CommandReplyType::None)
	{
		ReplyByteCount = 0;
		switch (CurrentCommand->ReplyType)
		{
			case CommandReplyType::uInt8:
				ReplyByteCountMax = 1;
				break;
			case CommandReplyType::uInt16:
				ReplyByteCountMax = 2;
				break;
			case CommandReplyType::uInt24:
				ReplyByteCountMax = 3;
				break;
			case CommandReplyType::SlitPositions:
				ReplyByteCountMax = 4;
				break;
			case CommandReplyType::SerialID:
				ReplyByteCountMax = 5;
				break;
			case CommandReplyType::GratingID:
				ReplyByteCountMax = 6;
				break;
			default:
				ReplyByteCountMax = 0;
				break;
		}
	}
}
void SPDKMonochromator::CheckForCommandReply()
{
	if (SerialPort->available())
	{
		uint8_t ByteRead = SerialPort->read();
		//Serial.print("MR:");
		//Serial.print(ByteRead);
		//Serial.print(";\n");
		bool ByteMatchesCommand = (ByteRead == CurrentCommand->CommandInt);
		if (ByteMatchesCommand)
		{
			if (CurrentCommand->Command == CommandType::Echo)
			{
				ModeTransitionToIdle();
			}
			else if (CurrentCommand->ResetAfter)
			{
				ResetTime = micros();
				Mode = ModeType::WaitToSendEcho;
			}
			else if (CurrentCommand->ReplyType != CommandReplyType::None)
			{
				ReplyByteCount = 0;
				Mode = ModeType::WaitForParameterReply;
				CommandReplyTime = micros();
			}
			else
			{
				if (CurrentCommand->SendType != CommandParameterType::None)
				{
					SendCommandParameter();
				}
				Mode = ModeType::WaitForStatus;
				CommandReplyTime = micros();
			}
		}
	}
	else if ( (micros() - LastCommandSentTime) > CommandReplyTimeMax)
	{
		Serial.print("<MONOERROR>(No command reply.)\n");
		ModeTransitionToIdle();
	}
}
void SPDKMonochromator::CheckForParameterReply()
{
	if (SerialPort->available())
	{
		uint8_t ByteRead = SerialPort->read();
		//Serial.print("MP:");
		//Serial.print(ReplyByteCount);
		//Serial.print(":");
		//Serial.print(ReplyByteCountMax);
		//Serial.print(":");
		//Serial.print(ByteRead);
		//Serial.print(";\n");
		ReplyByteCount++;
		ReplyData[ReplyByteCountMax - ReplyByteCount] = ByteRead;
		if (ReplyByteCount == ReplyByteCountMax)
		{
			ParseReplyData();
			Mode = ModeType::WaitForStatus;
		}
		CommandReplyTime = micros();
	}
	else if ( (micros() - CommandReplyTime) > CommandReplyTimeMax)
	{
		Serial.print("<MONOERROR>(No command reply.)\n");
		ModeTransitionToIdle();
	}
}
void SPDKMonochromator::ParseReplyData()
{
	ParameterConverter Converter;
	Converter.uInt32 = 0;
	switch (CurrentCommand->ReplyType)
	{
		case CommandReplyType::uInt8:
			Converter.uInt8Array[0] = ReplyData[0];
			UpdateInternalVariables(Converter.uInt32,CurrentCommand->Command);
			break;
		case CommandReplyType::uInt16:
			Converter.uInt8Array[0] = ReplyData[0];
			Converter.uInt8Array[1] = ReplyData[1];
			UpdateInternalVariables(Converter.uInt32,CurrentCommand->Command);
			break;
		case CommandReplyType::uInt24:
			Converter.uInt8Array[0] = ReplyData[0];
			Converter.uInt8Array[1] = ReplyData[1];
			Converter.uInt8Array[2] = ReplyData[2];
			UpdateInternalVariables(Converter.uInt32,CurrentCommand->Command);
			break;
		case CommandReplyType::SlitPositions:
			Converter.uInt8Array[0] = ReplyData[0];
			Converter.uInt8Array[1] = ReplyData[1];
			Converter.uInt8Array[2] = ReplyData[2];
			Converter.uInt8Array[3] = ReplyData[3];
			SlitPositionEnterance = Converter.uInt16Array[1];
			SlitPositionExit = Converter.uInt16Array[0];
			break;
		case CommandReplyType::SerialID:
			IDNumber[0] = ReplyData[0];
			IDNumber[1] = ReplyData[1];
			IDNumber[2] = ReplyData[2];
			IDNumber[3] = ReplyData[3];
			IDNumber[4] = ReplyData[4];
			break;
		case CommandReplyType::GratingID:
			GratingsInstalled = ReplyData[5];
			GratingCurrent = ReplyData[4];
			Converter.uInt8Array[0] = ReplyData[0];
			Converter.uInt8Array[1] = ReplyData[1];
			Converter.uInt8Array[2] = ReplyData[2];
			Converter.uInt8Array[3] = ReplyData[3];
			GratingCurrentRuling = Converter.uInt16Array[1];
			GratingCurrentBlaze = Converter.uInt16Array[0];
			break;
		default:
			break;
	}
}
void SPDKMonochromator::UpdateInternalVariables(uint32_t NewValue, CommandType PropertyToUpdate)
{
	switch (PropertyToUpdate)
	{
		case CommandType::ScanSpeedGet:
			ScanSpeed = NewValue;
			break;
		case CommandType::WavelengthGet:
			Wavelength = (float)(NewValue)/100.0;
			break;
		default:
			Serial.print("<MONOERROR>(Error updating internal variables.)\n");
			break;
	}
}
void SPDKMonochromator::ModeTransitionToIdle()
{
	if (CommandQueueEmpty())
	{
		Busy = false;
		if (RecievedCallback != NULL)
		{
			RecievedCallback(CurrentCommand->Command);
		}
	}
	Mode = ModeType::Idle;
}
void SPDKMonochromator::CheckForStatus()
{
	if (SerialPort->available())
	{
		StatusByte = SerialPort->read();
		//Serial.print("MS:");
		//Serial.print(StatusByte);
		//Serial.print(";\n");
		bool ParameterNotAccepted = bitRead(StatusByte,static_cast<uint8_t>(StatusByteMeaning::GoodBadValue));
		bool ParameterIsAlreadyEqual = bitRead(StatusByte,static_cast<uint8_t>(StatusByteMeaning::IsEqualToCurrent));
		if (ParameterNotAccepted && !ParameterIsAlreadyEqual)
		{
			Serial.print("<MONOERROR>(Status indicated parameter not accepted.)\n");
		}
		if (CurrentCommand->Command == CommandType::Echo)
		{
			ModeTransitionToIdle();
		}
		else if (CurrentCommand->Command == CommandType::Reset)
		{
			ResetTime = micros();
			Mode = ModeType::WaitToSendEcho;
		}
		else
		{
			Mode = ModeType::WaitForCompleted;
			StatusByteReceivedTime = micros();
		}
	}
	else if ( (micros() - CommandReplyTime) > CommandReplyTimeMax)
	{
		Serial.print("<MONOERROR>(No status byte received.)\n");
		ModeTransitionToIdle();
	}
}
void SPDKMonochromator::CheckForCompleted()
{
	if (SerialPort->available())
	{
		uint8_t ByteRead = SerialPort->read();
		//Serial.print("MC:");
		//Serial.print(ByteRead);
		//Serial.print(";\n");
		bool ByteMatchesCompleted = (ByteRead == CompeletedByte);
		if (!ByteMatchesCompleted)
		{
			Serial.print("<MONOERROR>(Unexpected byte received waiting for complete.)\n");
		}
		ModeTransitionToIdle();
	}
	else if ( (micros() - StatusByteReceivedTime) > CurrentCommandTimeToComplete)
	{
		Serial.print("<MONOERROR>(Timeout waiting for command to complete.)\n");
		ModeTransitionToIdle();
	}
}
void SPDKMonochromator::WaitToSendEcho()
{
	if ( (micros() - ResetTime) > ResetCompleteTime)
	{
		CommandStruct* EchoCommand = const_cast<CommandStruct*>(&CommandLibrary[static_cast<uint8_t>(CommandType::Echo)]);
		SendCommand(EchoCommand);
		Mode = ModeType::WaitForCommandReply;
	}
}
bool SPDKMonochromator::IsBusy()
{
	return Busy;
}
bool SPDKMonochromator::SendCommand(CommandStruct* CommandToSend)
{
	bool Status = true;
	if (CommandToSend->Command == CommandType::None)
	{
		Serial.print("<MONOERROR>(Invalid command in buffer.)\n");
		Status = false;
	}
	else if (CommandToSend->Command == CommandType::Reset)
	{
		SerialPort->write(CurrentCommand->CommandInt);
		SerialPort->write(CurrentCommand->CommandInt);
		SerialPort->write(CurrentCommand->CommandInt);
		LastCommandSentTime = micros();
	}
	else
	{
		SerialPort->write(CurrentCommand->CommandInt);
		//Serial.print("MO:");
		//Serial.print(CurrentCommand->CommandInt);
		//Serial.print(";\n");
		LastCommandSentTime = micros();
	}
	return Status;
}
void SPDKMonochromator::SendCommandParameter()
{
	ParameterConverter Converter;
	Converter.uInt32 = CurrentCommandParameter;
	//Serial.print("MK:");
	switch (CurrentCommand->SendType)
	{
		case(CommandParameterType::uInt24):
			SerialPort->write(Converter.uInt8Array[2]);
			//Serial.print(Converter.uInt8Array[2]);
			//Serial.print(",");
		case(CommandParameterType::uInt16):
			SerialPort->write(Converter.uInt8Array[1]);
			//Serial.print(Converter.uInt8Array[1]);
			//Serial.print(",");
		case(CommandParameterType::uInt8):
			SerialPort->write(Converter.uInt8Array[0]);
			//Serial.print(Converter.uInt8Array[0]);
			//Serial.print(",");
			break;
		default:
			Serial.print("<MONOERROR>(Attempt to send invalid parameter.)\n");
			break;
	}
	//Serial.print(";\n");
}
