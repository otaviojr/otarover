# @file   otarover_cli_test.py
# @author Otavio Ribeiro
# @date   10 Jan 2018
# @brief  Simle command line application to test the otarover daemon
#
# Copyright (c) 2018 Otávio Ribeiro <otavio.ribeiro@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

import socket;
import sys;
import curses;

from otarover_protocol import *;

MAX_SPEED=90;
MIN_SPEED=25;
ANGLE_VAR=10;
SPEED_VAR=5;

screen = curses.initscr();
curses.start_color();

curses.init_color(99,200,200,200);
curses.init_pair(1, curses.COLOR_WHITE, 99);
curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_RED);

screen.bkgd(' ', curses.color_pair(1));
screen.clear();
screen_size = screen.getmaxyx();
curses.curs_set(0);

def update_vars(win2,win3, current_speed, current_direction):
	label = 'cur. speed: %s' % (current_speed);
	win2.clear();
	win2.box();
	win2.addstr(1, int((screen_size[1]/4)/2) - int(len(label)/2),  label);
	win2.refresh();
	label = 'cur. direction: %sdeg' % (current_direction);
	win3.clear();
	win3.box();
	win3.addstr(1, int((screen_size[1]/4)/2) - int(len(label)/2),  label);
	win3.refresh();


def main(win):
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
	server_addr = ('192.168.0.47',7777);

	current_direction = 0;
	current_speed = 0;

	win1 = curses.newwin(8, screen_size[1], screen_size[0]-8, 0);
	win1.bkgd(' ', curses.color_pair(2));
	win1.box();
	win1.refresh();

	win2 = curses.newwin(3, int(screen_size[1]/4), 0, 0);
	win2.bkgd(' ', curses.color_pair(2));
	win2.box();
	win2.refresh();

	win3 = curses.newwin(3, int(screen_size[1]/4), 0, screen_size[1]-int(screen_size[1]/4));
	win3.bkgd(' ', curses.color_pair(2));
	win3.box();
	win3.refresh();

	update_vars(win2,win3,current_speed,current_direction);

	while 1:
		try:
			key = screen.getch();
			win1.clear();
			win1.box();
			line = 2;
			col = 2
			win1.addstr(line,col,"Detected key: ");
			win1.addstr(str(key));
			win1.addstr(" - ");
			win1.addstr(chr(key));
			line += 1;
			win1.refresh();

			message = None;
			message1 = None;

			if key == 27 or key == 113: #ESC
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
				win1.addstr(line, col, "Invalid key");
				line += 1

			win1.addstr(line, col, "Sending Message");
			line += 1;
			if message != None:
				sent = sock.sendto(message.encode(),server_addr);
				win1.addstr(line, col, 'sent %s bytes back to %s' % (sent, server_addr) );
				line += 1;
			else:
				win1.addstr(line, col, 'Error sending message');
				line += 1;

			if message1 != None:
				sent = sock.sendto(message1.encode(),server_addr);
				win1.addstr(line, col, 'sent %s bytes back to %s' % (sent, server_addr) );
				line += 1;

			win1.refresh();

			update_vars(win2,win3,current_speed,current_direction);

		except Exception as e:
			win1.addstr(line, col, str(e));
			win1.refresh();
			pass

	sock.close();
	curses.endwin();
	curses.curs_set(1);

curses.wrapper(main);
