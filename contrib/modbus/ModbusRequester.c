#include <ModbusRequester.h>
#include "ModbusDefinitions.h"
#include "ModbusMaster.h"
#include "Log.h"
#include "Crc.h"
//This function sends the message
int requester_request(ModbusMaster *mb, Uint16 slaveAddress, ModbusFunctionCode functionCode,
		Uint16 addr, Uint16 totalData) {
	mb->requester.slaveAddress = slaveAddress;//load data to modbus master
	mb->requester.functionCode = functionCode;//this is the data to send
	mb->requester.addr	       = addr;        //
	mb->requester.totalData    = totalData;   //mb here is a pointer, thus ->
	mb->requester.generate(mb);               //generate the to send data

	// Set the ModbusMaster to the start state
	mb->state = MB_START;

	// Wait until the MobbusMaster finish all the state flow
	while(mb->state != MB_END) {
		mb->loopStates(mb); //this matters
	}

	return 1;
}

int requester_readInputs(ModbusMaster *mb, Uint16 slaveAddress, Uint16 addr, Uint16 totalData) {
	return mb->requester.request(mb, slaveAddress, MB_FUNC_READ_INPUT, addr, totalData);
}

int requester_readCoils(ModbusMaster *mb, Uint16 slaveAddress, Uint16 addr, Uint16 totalData) {
	return mb->requester.request(mb, slaveAddress, MB_FUNC_READ_COIL, addr, totalData);
}

int requester_readHolding(ModbusMaster *mb, Uint16 slaveAddress, Uint16 addr, Uint16 totalData) {
	return mb->requester.request(mb, slaveAddress, MB_FUNC_READ_HOLDINGREGISTERS, addr, totalData);
}

int requester_readInputRegs(ModbusMaster *mb, Uint16 slaveAddress, Uint16 addr, Uint16 totalData) {
	return mb->requester.request(mb, slaveAddress, MB_FUNC_READ_INPUTREGISTERS, addr, totalData);
}

int requester_forceCoils(ModbusMaster *mb, Uint16 slaveAddress, Uint16 addr, Uint16 totalData) {
	return mb->requester.request(mb, slaveAddress, MB_FUNC_FORCE_NCOILS, addr, totalData);
}

int requester_writeHolding(ModbusMaster *mb, Uint16 slaveAddress, Uint16 addr, Uint16 totalData) {
	return mb->requester.request(mb, slaveAddress, MB_FUNC_WRITE_NREGISTERS, addr, totalData);
}
//generate the tobe transmited data according the modbus protocol
//this one is important
void requester_generate(ModbusMaster *master) {
	//requester is a member of master, and we need to deal with requester
	//so we define a requester and load the value from master requester
	//and nothing is changed in the master,by doing it this way.
	ModbusRequester requester = master->requester;
	Uint16 i, sizeWithoutCRC;
	Uint16 * transmitStringWithoutCRC;

	// Reference to MODBUS Data Map
	char * dataPtr;
//	Uint16 sizeOfMap = 0;
// why this?
// it should be that the master already has the address and the function code
// requester just copyed the data!
	master->dataRequest.slaveAddress = requester.slaveAddress;
	master->dataRequest.functionCode = requester.functionCode;

	// First, get the right data map based on the function code
	if (requester.functionCode == MB_FUNC_READ_COIL ||
			requester.functionCode == MB_FUNC_FORCE_COIL ||
			requester.functionCode == MB_FUNC_FORCE_NCOILS)
	{
		dataPtr = (char *)&(master->coils);
//		sizeOfMap = sizeof(master->coils);
	}
	else if (requester.functionCode == MB_FUNC_READ_HOLDINGREGISTERS ||
			requester.functionCode == MB_FUNC_WRITE_HOLDINGREGISTER ||
			requester.functionCode == MB_FUNC_WRITE_NREGISTERS)
	{
		dataPtr = (char *)&(master->holdingRegisters);
//		sizeOfMap = sizeof(master->holdingRegisters);
	}

	// Second: prepare the dataRequest content array
	if (requester.functionCode == MB_FUNC_READ_COIL || requester.functionCode == MB_FUNC_READ_INPUT ||
			requester.functionCode == MB_FUNC_READ_HOLDINGREGISTERS || requester.functionCode == MB_FUNC_READ_INPUTREGISTERS)
	{
		master->dataRequest.content[MB_READ_ADDRESS_HIGH]   = (requester.addr & 0xFF00) >> 8;
		master->dataRequest.content[MB_READ_ADDRESS_LOW]    = (requester.addr & 0x00FF);
		master->dataRequest.content[MB_READ_TOTALDATA_HIGH] = (requester.totalData & 0xFF00) >> 8;
		master->dataRequest.content[MB_READ_TOTALDATA_LOW]  = (requester.totalData & 0x00FF);
		master->dataRequest.contentIdx = MB_READ_TOTALDATA_LOW + 1;
	}
	else if (requester.functionCode == MB_FUNC_WRITE_HOLDINGREGISTER || requester.functionCode == MB_FUNC_FORCE_COIL) {
		master->dataRequest.content[MB_WRITE_ADDRESS_HIGH]  = (requester.addr & 0xFF00) >> 8;
		master->dataRequest.content[MB_WRITE_ADDRESS_HIGH]  = (requester.addr & 0x00FF);

		// Get the data at the specified address (one byte only)
		#if MB_32_BITS_REGISTERS == true
		master->dataRequest.content[MB_WRITE_VALUE_HIGH] = (*(dataPtr + requester.addr + 1) & 0xFF00) >> 8;
		master->dataRequest.content[MB_WRITE_VALUE_LOW]  = (*(dataPtr + requester.addr + 1) & 0x00FF);
		#else
		master->dataRequest.content[MB_WRITE_VALUE_HIGH] = (*(dataPtr + requester.addr) & 0xFF00) >> 8;
		master->dataRequest.content[MB_WRITE_VALUE_LOW]  = (*(dataPtr + requester.addr) & 0x00FF);
		#endif

		master->dataRequest.contentIdx = MB_WRITE_VALUE_LOW + 1;
	}
	else if (requester.functionCode == MB_FUNC_WRITE_NREGISTERS ||	requester.functionCode == MB_FUNC_FORCE_NCOILS) {
		master->dataRequest.content[MB_WRITE_N_ADDRESS_HIGH]  = (requester.addr & 0xFF00) >> 8;
		master->dataRequest.content[MB_WRITE_N_ADDRESS_LOW]  = (requester.addr & 0x00FF);

		master->dataRequest.content[MB_WRITE_N_QUANTITY_HIGH] = (requester.totalData & 0xFF00) >> 8;
		master->dataRequest.content[MB_WRITE_N_QUANTITY_LOW]  = (requester.totalData & 0x00FF);
		master->dataRequest.content[MB_WRITE_N_BYTES]         = (requester.totalData) * 2;
		master->dataRequest.contentIdx = MB_WRITE_N_VALUES_START_HIGH;

		// Loop throught the selected data map
		for(i=0; i < requester.totalData; i++) {
			Uint16 padding = i + requester.addr;
			#if MB_32_BITS_REGISTERS == true
			master->dataRequest.content[master->dataRequest.contentIdx++] = (*(dataPtr + padding + 1) & 0xFF00) >> 8;
			master->dataRequest.content[master->dataRequest.contentIdx++] = (*(dataPtr + padding + 1) & 0x00FF);
			master->dataRequest.content[master->dataRequest.contentIdx++] = (*(dataPtr + padding + 0) & 0xFF00) >> 8;
			master->dataRequest.content[master->dataRequest.contentIdx++] = (*(dataPtr + padding + 0) & 0x00FF);
			i++;
			#else
			master->dataRequest.content[master->dataRequest.contentIdx++] = (*(dataPtr + padding) & 0xFF00) >> 8;
			master->dataRequest.content[master->dataRequest.contentIdx++] = (*(dataPtr + padding) & 0x00FF);
			#endif
		}
	}

	// Prepares data request string
	master->dataRequest.size = MB_SIZE_COMMON_DATA + master->dataRequest.contentIdx;
	sizeWithoutCRC = master->dataRequest.size - 2;
	transmitStringWithoutCRC = master->dataResponse.getTransmitStringWithoutCRC(&master->dataRequest);
	master->dataRequest.crc = generateCrc( transmitStringWithoutCRC, sizeWithoutCRC, true );

	master->requester.isReady = true;
}
//what is this?
void requester_save(ModbusMaster *master) {
	ModbusRequester requester = master->requester;

	// Reference to MODBUS Data Map
	char * dataPtr;
//	Uint16 sizeOfMap;

	// First, get the right data map based on the function code
	if (requester.functionCode == MB_FUNC_READ_COIL ||
			requester.functionCode == MB_FUNC_FORCE_COIL ||
			requester.functionCode == MB_FUNC_FORCE_NCOILS)
	{
		dataPtr = (char *)&(master->coils);
//		sizeOfMap = sizeof(master->coils);
	}
	else if (requester.functionCode == MB_FUNC_READ_HOLDINGREGISTERS ||
			requester.functionCode == MB_FUNC_WRITE_HOLDINGREGISTER ||
			requester.functionCode == MB_FUNC_WRITE_NREGISTERS)
	{
		dataPtr = (char *)&(master->holdingRegisters);
//		sizeOfMap = sizeof(master->holdingRegisters);
	}

	if (requester.functionCode == MB_FUNC_READ_COIL || requester.functionCode == MB_FUNC_READ_INPUT ||
				requester.functionCode == MB_FUNC_READ_HOLDINGREGISTERS || requester.functionCode == MB_FUNC_READ_INPUTREGISTERS)
	{
		Uint16 * memAddr;
		Uint16 i;

		for(i=0; i < (requester.totalData); i++) {
			Uint16 padding = i + requester.addr;
			memAddr = (Uint16 *) (dataPtr + padding);
			*(memAddr) = (master->dataResponse.content[1+i*2] << 8) | (master->dataResponse.content[(1+1)+i*2]);
		}
	}
}
// so this requesthandler is just long for requester
ModbusRequester construct_ModbusRequestHandler(){
	ModbusRequester requester;

	requester.slaveAddress = 1;
	requester.functionCode = MB_FUNC_READ_HOLDINGREGISTERS;
	requester.addr = 0;
	requester.totalData = 0;

	requester.isReady = false;
// function pointers point to functions
	requester.generate   = requester_generate;
	requester.save       = requester_save;
	requester.request    = requester_request;

	requester.readCoils     = requester_readCoils;
	requester.readInputs    = requester_readInputs;
	requester.readHolding   = requester_readHolding;
	requester.readInputRegs = requester_readInputRegs;
	requester.writeHolding  = requester_writeHolding;
	requester.forceCoils    = requester_forceCoils;

	MB_DATA_HANDLER_DEBUG();

	return requester;
}
