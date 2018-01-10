import socket;
import sys;
import curses;

from otarover_protocol import *;

MAX_SPEED=40;
MIN_SPEED=25;
ANGLE_VAR=10;
SPEED_VAR=5;

def main(win):
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
	server_addr = ('192.168.0.47',7777);

	current_direction = 0;
	current_speed = 0;

	while 1:
		try:
			key = win.getch();
			win.clear();
			win.addstr("Detected key: ");
			win.addstr(str(key));
			win.addstr(" - ");
			win.addstr(chr(key));
			win.addstr("\n");

			message = None;
			message1 = None;

			if key == 27: #ESC
				sys.exit();
			elif key == 97: #a
				current_direction -= ANGLE_VAR;
				if current_direction < 0:
					current_direction += 360;

				message = OtaRoverProtocolMessage(OtaRoverProtocolMessageType.OTAROVER_PROTOCOL_MESSAGE_TYPE_MOV, OtaRoverProtocolCmd.OTAROVER_PROTOCOL_CMD_DIRECTION, OtaRoverProtocolValueType.OTAROVER_PROTOCOL_VALUE_TYPE_INT32,current_direction);
			elif key == 100: #d
				current_direction += ANGLE_VAR;
				if current_direction >= 360:
					current_direction -= 360;

				message = OtaRoverProtocolMessage(OtaRoverProtocolMessageType.OTAROVER_PROTOCOL_MESSAGE_TYPE_MOV, OtaRoverProtocolCmd.OTAROVER_PROTOCOL_CMD_DIRECTION,OtaRoverProtocolValueType.OTAROVER_PROTOCOL_VALUE_TYPE_INT32,current_direction);
			elif key == 115: #s
				current_speed = 0;
				current_direction = 0;

				message = OtaRoverProtocolMessage(OtaRoverProtocolMessageType.OTAROVER_PROTOCOL_MESSAGE_TYPE_MOV, OtaRoverProtocolCmd.OTAROVER_PROTOCOL_CMD_SPEED,OtaRoverProtocolValueType.OTAROVER_PROTOCOL_VALUE_TYPE_INT32,current_speed);
				message1 = OtaRoverProtocolMessage(OtaRoverProtocolMessageType.OTAROVER_PROTOCOL_MESSAGE_TYPE_MOV, OtaRoverProtocolCmd.OTAROVER_PROTOCOL_CMD_DIRECTION,OtaRoverProtocolValueType.OTAROVER_PROTOCOL_VALUE_TYPE_INT32,current_direction);
			elif key == 120: #x
				current_speed -= SPEED_VAR;
				if current_speed < 0:
					current_speed = 0;

				message = OtaRoverProtocolMessage(OtaRoverProtocolMessageType.OTAROVER_PROTOCOL_MESSAGE_TYPE_MOV, OtaRoverProtocolCmd.OTAROVER_PROTOCOL_CMD_SPEED,OtaRoverProtocolValueType.OTAROVER_PROTOCOL_VALUE_TYPE_INT32,current_speed);
			elif key == 119: #w
				current_speed += SPEED_VAR;
				if current_speed > MAX_SPEED:
					current_speed = MAX_SPEED;

				message = OtaRoverProtocolMessage(OtaRoverProtocolMessageType.OTAROVER_PROTOCOL_MESSAGE_TYPE_MOV, OtaRoverProtocolCmd.OTAROVER_PROTOCOL_CMD_SPEED,OtaRoverProtocolValueType.OTAROVER_PROTOCOL_VALUE_TYPE_INT32,current_speed);
			elif key == 122: #z
				if current_speed == 0 and current_direction == 0:
					current_direction = 180;
				elif current_speed == 0 and current_direction == 180:
 					current_direction = 0;

				message = OtaRoverProtocolMessage(OtaRoverProtocolMessageType.OTAROVER_PROTOCOL_MESSAGE_TYPE_MOV, OtaRoverProtocolCmd.OTAROVER_PROTOCOL_CMD_DIRECTION,OtaRoverProtocolValueType.OTAROVER_PROTOCOL_VALUE_TYPE_INT32,current_direction);
			else:
				win.addstr("Invalid key\n");

			win.addstr("Sending Message\n");
			if message != None:
				sent = sock.sendto(message.encode(),server_addr);
				win.addstr( 'sent %s bytes back to %s' % (sent, server_addr) );
				win.addstr( '\n');
			else:
				win.addstr('Error sending message\n');

			if message1 != None:
				sent = sock.sendto(message1.encode(),server_addr);
				win.addstr( 'sent %s bytes back to %s' % (sent, server_addr) );
				win.addstr( '\n');

		except Exception as e:
			win.addstr(str(e));
			pass

	sock.close();

curses.wrapper(main);
