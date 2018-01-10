from enum import Enum;

class OtaRoverProtocolMessageType(Enum):
	OTAROVER_PROTOCOL_MESSAGE_TYPE_MOV=0x01
	OTAROVER_PROTOCOL_MESSAGE_TYPE_BAT=0x02

class OtaRoverProtocolValueType(Enum):
	OTAROVER_PROTOCOL_VALUE_TYPE_INT16=0x01
	OTAROVER_PROTOCOL_VALUE_TYPE_INT32=0x02
	OTAROVER_PROTOCOL_VALUE_TYPE_STRING=0x03

class OtaRoverProtocolCmd(Enum):
	OTAROVER_PROTOCOL_CMD_DIRECTION=0x01
	OTAROVER_PROTOCOL_CMD_SPEED=0x02

class OtaRoverProtocolMessage(object):
	DEFAULT_MAGIC=0x7777;

	def __init__(self, v_message_type, v_cmd, v_value_type, v_val):
		self.magic = OtaRoverProtocolMessage.DEFAULT_MAGIC;
		self.message_length = 0x00;
		self.message_type = v_message_type;
		self.cmd = v_cmd;
		self.value_type=v_value_type;
		self.val = v_val;

	def encode(self):
		msg=bytearray();
		msg.append((self.magic>>24)&0xFF);
		msg.append((self.magic>>16)&0xFF);
		msg.append((self.magic>>8)&0xFF);
		msg.append(self.magic&0xFF);
		msg.append(0);
		msg.append(0);
		msg.append(0);
		msg.append(0);
		msg.append((self.message_type.value>>8)&0xFF);
		msg.append(self.message_type.value&0xFF);
		msg.append((self.cmd.value>>8)&0xFF);
		msg.append(self.cmd.value&0xFF);
		msg.append((self.value_type.value>>8)&0xFF);
		msg.append(self.value_type.value&0xFF);

		if self.value_type == OtaRoverProtocolValueType.OTAROVER_PROTOCOL_VALUE_TYPE_INT16:
			msg.append((self.val>>8)&0xFF);
			msg.append(self.val&0xFF);
		elif self.value_type == OtaRoverProtocolValueType.OTAROVER_PROTOCOL_VALUE_TYPE_INT32:
			msg.append((self.val>>24)&0xFF);
			msg.append((self.val>>16)&0xFF);
			msg.append((self.val>>8)&0xFF);
			msg.append(self.val&0xFF);

		msg[4] = (len(msg)>>24)&0xFF;
		msg[5] = (len(msg)>>16)&0xFF;
		msg[6] = (len(msg)>>8)&0xFF;
		msg[7] = len(msg)&0xFF;
		return msg;
