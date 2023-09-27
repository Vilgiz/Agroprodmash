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
.PROGRAM Comment___ () ; Comments for IDE. Do not use.
	; @@@ PROJECT @@@
	; 
	; @@@ HISTORY @@@
	; 25.09.2023 11:38:47
	; 
	; @@@ INSPECTION @@@
	; @@@ CONNECTION @@@
	; KROSET R02
	; 127.0.0.1
	; 9205
	; @@@ PROGRAM @@@
	; 0:Teach:F
	; 0:autostart4.pc:B
	; .i 
	; 0:init.pc:B
	; .i 
	; 0:Calculate.PC:B
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
