#ifndef SPDKMonochromator_h	//check for multiple inclusions
#define SPDKMonochromator_h

#include "Arduino.h"

#define SPDKMonochromatorQueueCount 8

class SPDKMonochromator
{
	public:
		enum class CommandType : uint8_t
		{
			None,
			Clear,
			CSRMode,
			Echo,
			GratingCal,
			GratingID,
			GratingSel,
			Reset,
			SlitReset,
			Scan,
			ScanUp,
			ScanDown,
			Serial,
			SlewUp,
			SlewDown,
			SlitWidthGetAll,
			SlitWidthSetAll,
			SlitWidthSetInput,
			SlitWidthSetOutput,
			SlitCalInput,
			SlitCalOutput,
			ScanSpeedGet,
			ScanSpeedSet,
			StepDown,
			StepUp,
			SelfTest,
			WavelengthGet,
			WavelengthSet,
			WavelengthZero,
			ScanSlewStop,
		};
		enum class CommandParameterType : uint8_t
		{
			None,
			uInt8,
			uInt16,
			uInt24
		};
		enum class CommandReplyType : uint8_t
		{
			None,
			uInt8,
			uInt16,
			uInt24,
			GratingID,
			SerialID,
			SlitPositions
		};
		enum class StatusByteMeaning : uint8_t
		{
			NegPosOrder = 0,
			CSRActive = 2,
			NegPosMove = 4,
			TooSmallBig = 5,
			IsEqualToCurrent = 6,
			GoodBadValue = 7,
		};
		enum class ModeType : uint8_t {
			Inactive,
			Idle,
			WaitForCommandReply,
			WaitForParameterReply,
			WaitForStatus,
			WaitForCompleted,
			WaitToSendEcho
		};
		union ParameterConverter
		{
			uint32_t uInt32;
			uint8_t uInt8Array[4];
			uint16_t uInt16Array[2];
		};
		struct CommandStruct
		{
			CommandType Command;
			uint8_t CommandInt;
			CommandParameterType SendType;
			CommandReplyType ReplyType;
			bool ResetAfter;
			bool StartContinuousMovement;
			uint32_t TimeToComplete;
		};
		struct CommandQueueEntry
		{
			CommandStruct* Command;
			uint32_t Parameter;
		};
		typedef void ( *FinishedListener )(SPDKMonochromator::CommandType Command);
		SPDKMonochromator(HardwareSerial* serial); //Invoke with SPDKMonochromator(&SerialN);
		void SendSetSlitWidthAll(uint16_t SlitWidth);
		void SendSetSlitWidthInput(uint16_t SlitWidth);
		void SendSetSlitWidthOutput(uint16_t SlitWidth);
		void SendSetWavelength(float Wavelength);
		void SendGetWavelength();
		void SendSetGrating(uint8_t GratingNumber);
		void SendGetGrating();
		void SendGetSlitWidth();
		void SetRecievedCallback(FinishedListener Finished);
		uint8_t* GetIDNumber();
		float GetWavelength();
		uint16_t GetScanSpeed();
		uint8_t GetGratingsInstalled();
		uint8_t GetGratingCurrent();
		uint16_t GetGratingCurrentRuling();
		uint16_t GetGratingCurrentBlaze();
		uint16_t GetSlitWidthInput();
		uint16_t GetSlitWidthOutput();
		bool IsBusy();
		void Check();
		void Begin();
		float GetCurrentWavelength();
	private:
		void CheckCommandQueue();
		void Enqueue(CommandType Command);
		void Enqueue(CommandType Command, uint32_t Parameter);
		void ClearCommandQueue();
		bool SendCommand(CommandStruct* CommandToSend);
		bool CommandQueueFull();
		bool CommandQueueEmpty();
		uint8_t CommandQueueCount();
		void CommandQueueAdvance();
		void CommandQueueRetreat();
		void CommandQueuePut(CommandStruct* CommandPointer, uint32_t Parameter);
		bool CommandQueuePullToCurrentCommand();
		void UpdateCurrentCommandVariables();
		void CheckForCommandReply();
		void CheckForParameterReply();
		void ParseReplyData();
		void UpdateInternalVariables(uint32_t NewValue, CommandType PropertyToUpdate);
		void ModeTransitionToIdle();
		void CheckForStatus();
		void CheckForCompleted();
		void WaitToSendEcho();
		void SendCommandParameter();
		static const CommandStruct CommandLibrary[];
		static const uint8_t CompeletedByte;
		static const uint32_t ResetCompleteTime;
		static const uint32_t CommandReplyTimeMax;
		static const uint32_t TimeToCompleteDefault;
		static const uint8_t RetryCountMax;
		static const uint32_t WipeInputEvery;
		HardwareSerial* SerialPort;
		FinishedListener RecievedCallback;
		CommandStruct* CurrentCommand;
		uint32_t CurrentCommandParameter;
		uint32_t CurrentCommandTimeToComplete;
		uint32_t LastWipeTime = 0;
		uint8_t ReplyByteCount;
		uint8_t ReplyData[6];
		uint8_t ReplyByteCountMax;
		CommandQueueEntry CommandQueue[SPDKMonochromatorQueueCount];
		uint8_t CommandQueueHead;
		uint8_t CommandQueueTail;
		bool CommandQueueFullFlag;
		uint8_t StatusByte;
		ModeType Mode;
		bool Busy;
		uint16_t SlitWidthInput;
		uint16_t SlitWidthOutput;
		uint8_t GratingCurrent;
		uint32_t LastCommandSentTime;
		uint32_t CommandReplyTime;
		uint32_t StatusByteReceivedTime;
		uint32_t ResetTime;
		uint8_t ResetWaitCount;
		uint32_t CommandRetryCount;
		uint8_t IDNumber[5];
		uint16_t SlitPositionEnterance;
		uint16_t SlitPositionExit;
		float Wavelength;
		uint16_t ScanSpeed;
		uint8_t GratingsInstalled;
		uint8_t GratingsCurrent;
		uint16_t GratingCurrentRuling;
		uint16_t GratingCurrentBlaze;
};
#endif
