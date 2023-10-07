.AUXDATA
N_OX1    "O_MAGNET[1]  "
N_OX2    "O_MAGNET[2]  "
N_OX3    "O_MAGNET[3]  "
N_OX4    "O_MAGNET[4]  "
N_OX5    "O_MAGNET[5]  "
N_OX6    "O_MAGNET[6]  "
N_INT1    "IO_MAGNET_ON  "
.END
.PROGRAM Teach()
  BASE NULL
  TOOL NULL
  TOOL tool_calibr
  BREAK
  ALIGN
  LAPPRO O1,-100
  LMOVE O1
  LMOVE Y1
  CALL Calculate_X1.pc
  BREAK
  POINT base_coord = FRAME(O1,X1,Y1,O1)
  BASE base_coord
  BREAK
  BREAK
  LMOVE TRANS(-100,100,100,0,0,90)
  BREAK
  JMOVE #t_change
.END
.PROGRAM MAIN()
  ; *******************************************************************
  ;
  ; Program:      MAIN
  ; Comment:      
  ; Author:       User
  ;
  ; Date:         9/30/2023
  ;
  ; *******************************************************************
  ;
  CALL init.pc
.END
.PROGRAM init.pc ()
  ; *******************************************************************
  ;
  ; Program:      init.pc
  ; Comment:      
  ; Author:       User
  ;
  ; Date:         9/25/2023
  ;
  ; *******************************************************************
  ;
  FOR .i=1 TO 6
  O_MAGNET[.i]= start_mag_sig+.i-1
  END
  ; Internal signals for IFP
  int_exec_main = 2900
  ; TCP/IP Settings
  tcp_port = 48569
  $tcp_ip = "127.0.0.1"
  tcp_send_time = 10
  tcp_conn_time = 10
  tcp_recv_time = 15
  socket_id = -1
.END
.PROGRAM autostart4.pc ()
  ; *******************************************************************
  ;
  ; Program:      autostart4.pc
  ; Comment:      
  ; Author:       Shogun
  ;
  ; Date:         9/25/2023
  ;
  ; *******************************************************************
  ;
  WHILE TRUE DO
    IF sig(IO_MAGNET_ON)==TRUE
      FOR .i=1 TO 6
        SIGNAL O_MAGNET[.i]
      END
    IF SIG(IO_MAGNET_ON)== FALSE
      FOR .i=1 TO 6
        SIGNAL -O_MAGNET[.i]
      END
    END
    END
  END

.END
.PROGRAM Calculate.PC()
  .a = DX(O1)
  .aa = DY(O1)
  .b = DX(Y1)
  .bb = DY(Y1)
  .o = SQRT(10000*(.bb-.aa)*(.bb-.aa)*(.bb*.bb-2*.aa*.bb+.b*.b-2*.a*.b+.aa*.aa+.a*.a))
  .oo = (.bb*.bb-2*.aa*.bb+.b*.b-2*.a*.b+.aa*.aa+.a*.a)
  .x = .o/.oo+.a
  .y = .aa-((.b-.a)*(.x-.a))/(.bb-.aa)
  POINT X1 = TRANS(.x,.y,DZ(O1),0,0,0)
.END
.PROGRAM tcp_send.pc (.$msg) #164127
	.$send_buf[1] = .$msg
	.buf_n = 1
	TCP_SEND .ret, socket_id, .$send_buf[1], .buf_n, tcp_send_time
.END
.PROGRAM tcp_recv.pc (.$msg, .ret) #164379
	.num = 0
	.max_length = 255
	TCP_RECV .ret, socket_id, .$recv_buf[1], .num, tcp_recv_time, .max_length
	; Check data
	IF .ret < 0 THEN
		.$msg = ""
		.ret = -1
	ELSE
		IF .num > 0 THEN
			.$msg = .$recv_buf[1]
			CALL slog.pc ("Received message: " + .$msg)
		ELSE
			.$msg = ""
		END
	END
.END
.PROGRAM check_socket.pc () #1073;
	CALL slog.pc ("Checking tcp/ip status")
	TCP_STATUS .p1, .p2[0], .p3[0], .p4[0], .p5[0], .$p6[0]
	IF .p1 <> 0 THEN
		CALL slog.pc ("Found " + $ENCODE (.p1) + " active sockets:")
		FOR .i = 0 TO .p1 - 1
			CALL slog.pc ("1: " + " ---------->")
			CALL slog.pc ("Port: " + $ENCODE (.p2[.i]))
			CALL slog.pc ("Socket: " + $ENCODE (.p3[.i]))
			CALL slog.pc ("IP: " + .$p6[.i])
			;
			IF .p3[.i] <> 0 THEN
				TCP_CLOSE .ret, .p3[.i]
				CALL slog.pc ("Socket closed")
			END
			CALL slog.pc ("----------")
		END
	ELSE
		CALL slog.pc ("Tcp/ip status OK")
	END
.END
.PROGRAM open_socket.pc () #1053
	; Check socket
	CALL check_socket.pc
	; Get current IP
	.$tcp_adr = $tcp_ip
	FOR .i = 1 TO 4
		.$ip = $DECODE (.$tcp_adr, ".", 0)
		.ip[.i] = VAL (.$ip)
		IF .i < 4 THEN
			.$ip = $DECODE (.$tcp_adr, ".", 1)
		END
	END
	; Connecting
	CALL slog.pc ("Connecting to: " + $tcp_ip)
	.connect = TRUE
	.err_counter = 0
	WHILE .connect DO
		TCP_CONNECT socket_id, tcp_port, .ip[1], tcp_conn_time
		IF socket_id < 0 THEN
			IF .err_counter >= 5 THEN
				CALL slog.pc ("Number of connection attempts exceeded")
				socket_id = -1
				.connect = FALSE
			ELSE
				.err_counter = .err_counter + 1
				CALL slog.pc ("Connection failed. Attempt: " + $ENCODE (.err_counter))
			END
		ELSE
			CALL slog.pc ("Connection successfull with socket id: " + $ENCODE (socket_id))
			.connect = FALSE
		END
	END
.END
.PROGRAM close_socket.pc () #226
	CALL slog.pc ("Closing connection")
	TCP_CLOSE .ret, socket_id
	CALL slog.pc ("Connection closed")
.END
.PROGRAM Comment___ () ; Comments for IDE. Do not use.
	; @@@ PROJECT @@@
	; 
	; @@@ HISTORY @@@
	; 25.09.2023 11:38:47
	; 
	; @@@ INSPECTION @@@
	; @@@ CONNECTION @@@
	; KROSET R01
	; 127.0.0.1
	; 9105
	; @@@ PROGRAM @@@
	; 0:Teach:F
	; .pc 
	; 0:MAIN:F
	; 0:init.pc:B
	; .i 
	; 0:autostart4.pc:B
	; .i 
	; 0:Calculate.PC:B
	; .a 
	; .aa 
	; .b 
	; .bb 
	; .o 
	; .oo 
	; .x 
	; .y 
	; 0:tcp_send.pc:B
	; .buf_n 
	; .ret 
	; 0:tcp_recv.pc:B
	; .ret 
	; .num 
	; .max_length 
	; 0:check_socket.pc:B
	; .p1 
	; .p2 
	; .p3 
	; .p4 
	; .p5 
	; .i 
	; .ret 
	; 0:open_socket.pc:B
	; .i 
	; .ip 
	; .connect 
	; .err_counter 
	; 0:close_socket.pc:B
	; @@@ TRANS @@@
	; @@@ JOINTS @@@
	; @@@ REALS @@@
	; @@@ STRINGS @@@
	; @@@ INTEGER @@@
	; @@@ SIGNALS @@@
	; O_MAGNET[] 
	; IO_MAGNET_ON 
	; @@@ TOOLS @@@
	; @@@ BASE @@@
	; @@@ FRAME @@@
	; @@@ BOOL @@@
.END
.REALS
O_MAGNET[1] = 1
O_MAGNET[2] = 2
O_MAGNET[3] = 3
O_MAGNET[4] = 4
O_MAGNET[5] = 5
O_MAGNET[6] = 6
IO_MAGNET_ON = 2001
start_mag_sig = 13
.END
