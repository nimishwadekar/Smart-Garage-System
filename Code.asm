; This file is only for the hardware design

#make_bin#

#LOAD_SEGMENT=FFFFh#
#LOAD_OFFSET=0000h#

#CS=0000h#
#IP=0000h#
#DS=0000h#	
#ES=0000h#	
#SS=0000h#	
#SP=FFFEh#

#AX=0000h#
#BX=0000h#
#CX=0000h#
#DX=0000h#
#SI=0000h#
#DI=0000h#
#BP=0000h#

; 8255 (1)        
port1A      equ     00h         ; LCD data lines (o/p)
port1C      equ     04h         ; 0-2: LCD RS, RW, E  (o/p)
                                ; 4-6: Motor sleep', step, dir (o/p)
port1CR     equ     06h         ; Control register   

; 8255 (2)
port2A      equ     08h         ; ADC data lines (i/p)
port2C      equ     0Ch         ; 0-1: IREDs (o/p)
                                ; 2-3: ADC WR', RD' pins (o/p)
                                ; 4-5: Gate lines of counters 0 and 1
port2CR     equ     0Eh         ; Control register

; 8259
pri0        equ     10h
pri1        equ     12h

; 8253
counter0    equ     18h         ; Counter 0
counter1    equ     1Ah         ; Counter 1
counter2    equ     1Ch         ; Counter 2
counterCR   equ     1Eh         ; Control register
            
            jmp st1
            nop
            dd      00000000h   ; int 1h
            
            dw      adc_isr     ; nmi - int 2h
            dw      0000h
            
            db      244 dup(0)  ; int 3h - int 3Eh
            
            dw      button_isr  ; int 40h
            dw      0000h
            
            dw      ir1_isr     ; int 41h
            dw      0000h
            
            dw      ir2_isr     ; int 42h
            dw      0000h
            
            dw      timer_isr   ; int 43h
            dw      0000h
            
            db      752 dup(0)  

; Macro to put the motor to sleep
motor_sleep macro
            mov al, 08h         ; Sleep' pin low
            out port1CR, al
            endm
                 
                 
; Macro to wake up the motor from sleep                
motor_wake  macro
            mov al, 09h         ; Sleep' pin high
            out port1CR, al
            call delay_1ms      ; Delay to initialise motor after sleep      
            endm
                  
                  
; Macro to start the 5-minute timer
timer_begin macro
            mov al, 09h         ; Gate 0 low-high edge
            out port2CR, al
            mov al, 0Bh         ; Gate 1 low-high edge
            out port2CR, al
            endm


; Macro to stop the 5-minute timer
timer_end   macro
            mov al, 08h         ; Gate 0 high-low edge
            out port2CR, al
            mov al, 0Ah         ; Gate 1 high-low edge
            out port2CR, al
            endm            
                 
                 
st1:                 
            cli
            mov ax, 0100h       ; Initialising segment registers       
            mov ds, ax		
            mov es, ax
            mov ss, ax
            mov sp, 0FFFEh
            mov si, 0000h
        
            mov al, 10000000b   ; Initialising the 8255s I/O ports
            out port1CR, al
            mov al, 10010000b
            out port2CR, al
        
            mov al, 00010011b   ; Initialising the 8259
            out pri0, al
            mov al, 01000000b   ; Base address 40h
            out pri1, al
            mov al, 00000011b
            out pri1, al
            mov al, 11110000b
            out pri1, al
            mov al, 10000000b
            out pri0, al
            
            timer_end           ; Initialising the 8253 counters
            mov al, 00110100b
            out counterCR, al
            mov al, 01110100b
            out counterCR, al
            mov al, 50h         ; 50,000d = C350h
            out counter0, al
            mov al, 0C3h
            out counter0, al
            mov al, 30h         ; 30,000d = 7530h
            out counter1, al
            mov al, 75h
            out counter1, al 
        
            mov al, 38h       ; Initialising the LCD
            call lcd_cmd
            mov al, 38h
            call lcd_cmd
            mov al, 0Fh
            call lcd_cmd
            mov al, 06h
            call lcd_cmd
            mov al, 01h
            call lcd_cmd
            mov al, 0Ch
            call lcd_cmd
            mov al, 80h
            call lcd_cmd
            mov ax, cars
            call lcd_write
            
            motor_sleep         ; Putting motor to sleep until gate opens
            
            mov al, 07h         ; High RD' and low WR' to initialise ADC
            out port2CR, al
            mov al, 04h
            out port2CR, al
        
            mov al, 01h         ; Enabling the IREDs
            out port2CR, al
            mov al, 03h
            out port2CR, al

	    sti 
            
inf_loop:   jmp inf_loop        ; Wait for interrupts

              
              
; Procedure for approximately little over 1 ms delay
delay_1ms   proc near
            mov cx, 420         ; approximately little over 1 ms delay (420 * 3 memr @ 5MHz)
dec_cnt:    dec cx              
            jnz dec_cnt
            ret
delay_1ms   endp


; Procedure for lcd delay
delay_lcd   proc near
            mov cx, 200        
dec_c_lcd:  loop dec_c_lcd
            ret
delay_lcd   endp


; Procedure to send command to LCD, AL - command      
lcd_cmd     proc near
            push cx
            mov bl, al
            mov al, 05h         ; Set E to high
            out port1CR, al
            mov al, 02h         ; Set WR' to low
            out port1CR, al
            call delay_lcd
            mov al, 00h         ; Set RS to low
            out port1CR, al
            call delay_lcd
            mov al, bl
            out port1A, al      ; Send data to data lines
            mov al, 04h         ; Set E to low
            out port1CR, al
            pop cx
            ret
lcd_cmd     endp


; Procedure to send data to LCD, AL - data        
lcd_data    proc near
            push cx
            mov bl, al
            mov al, 05h         ; Set E to high
            out port1CR, al
            mov al, 02h         ; Set WR' to low
            out port1CR, al
            call delay_lcd
            mov al, 01h         ; Set RS to high
            out port1CR, al
            call delay_lcd
            mov al, bl
            out port1A, al      ; Send data to data lines
            mov al, 04h         ; Set E to low
            out port1CR, al
            pop cx
            ret
lcd_data    endp


; Procedure to write a number (or string if empty or full) to LCD, AX - number
lcd_write   proc near
            mov di, ax
            mov al, 01h       	; Clear screen
            call lcd_cmd
            mov al, 81h       	; Cursor to line 1, position 1
            call lcd_cmd
            lea si, empty_msg
            mov cl, 5
            cmp di, 0
            je empty_lcd
            lea si, full_msg
            mov cl, 4
            cmp di, 2000
            je full_lcd
            
            lea si, cars_dec	; Convert number of cars from hex to dec
            add si, 3
            mov dl, 10
            mov ax, di
            mov cl, 4
next_digit: div dl	
            mov dh, al
            mov al, ah
            or al, 30h
            mov [si], al
            dec si
            xor ah, ah
            mov al, dh
            dec cl
            jnz next_digit
            
            lea si, cars_dec	; Display number of cars
            mov cl, 4
num_lcd:    lodsb
            call lcd_data
            dec cl
            jnz num_lcd
            jmp wr_end
            
empty_lcd:  lodsb		; Display 'EMPTY'
            call lcd_data
            dec cl
            jnz empty_lcd
            jmp wr_end
            
full_lcd:   lodsb		; Display 'FULL'
            call lcd_data
            dec cl
            jnz full_lcd
            
wr_end:     ret
lcd_write   endp


; Procedure for one full rotation of motor opening; AH = 0 (open/clockwise), 1 (close, counterclockwise); 
motor_rot   proc near 
            cmp ah, 0           ; branch according to direction
            jne close
            mov al, 0Ch
            out port1CR, al
	    mov cl, 200
            jmp rot
close:      mov al, 0Dh
            out port1CR, al
            mov cl, 200         ; 200 steps for 1 rotaion (360 / 1.8)
rot:        mov al, 0Bh         ; step pulse high
            out port1CR, al
            mov al, 0Ah         ; step pulse low
            out port1CR, al
            dec cl
            jnz rot    
            ret
motor_rot   endp


; Procedure to begin AD conversion
adc_begin   proc near
            mov al, 05h         ; WR' low to high edge
            out port2CR, al
            ret
adc_begin   endp


; ADC Interrupt (int 2h - NMI)
adc_isr     proc near
            mov al, 06h         ; RD' high
            out port2CR, al
            in al, port2A
            cmp al, 0Bh         ; 11/255 = approx 0.04, 0.04 * 5000kg = 200kg - lower weight limit of car for garage
            jl nmi_end
            
            mov dl, direction	; Increment or decrement based on direction
            cmp dl, 00h
            jne chk_entry
            mov ax, cars
            dec ax
            mov cars, ax
            call lcd_write
            jmp nmi_end
            
chk_entry:  cmp dl, 0FFh
            jne nmi_end
            mov ax, cars
            inc ax
            mov cars, ax
            call lcd_write
            
nmi_end:    mov al, 07h         ; RD' high
            out port2CR, al
            mov al, 04h         ; WR' high to low edge
            out port2CR, al
            mov dl, 11h         ; Resetting direction to null
            mov direction, dl                        
            iret
adc_isr     endp


; Button Interrupt (To open/close garage door) (int 40h)
button_isr  proc near
            mov dl, door
            cmp dl, 00h		; Branch according to status of door
            jne int40_cls
            motor_wake		; Wake motor
            mov ah, 0
            call motor_rot	; Rotate motor
            motor_sleep		; Sleep motor
            timer_begin		; Start 5 minute timer to close the door
            jmp int40_end                                          
            
int40_cls:  timer_end		; Reset the timer if in progress
            motor_wake
            mov ah, 1
            call motor_rot
            motor_sleep
            
int40_end:  not dl		; Update status of door
            mov door, dl          
            iret
button_isr  endp 


; IR Sensor 1 Interrupt (int 41h)
ir1_isr     proc near
            mov dl, sensor1
            not dl		; Update sensor status to IR line broken
            mov sensor1, dl
            mov dl, sensor2
            cmp dl, 00h
            jne start_wt1	; Start AD conversion if car has not yet triggered other sensor
            mov dl, 00h		; Set direction variable if both sensors have been triggered
            mov direction, dl
	    jmp int41_end
start_wt1:  call adc_begin
int41_end:  iret
ir1_isr     endp


; IR Sensor 2 Interrupt (int 42h)
ir2_isr     proc near
            mov dl, sensor2
            not dl		; Update sensor status to IR line broken
            mov sensor2, dl
            mov dl, sensor1
            cmp dl, 00h
            jne start_wt2	; Start AD conversion if car has not yet triggered other sensor
            mov dl, 0FFh	; Set direction variable if both sensors have been triggered
            mov direction, dl
	    jmp int42_end
start_wt2:  call adc_begin
int42_end:  iret
ir2_isr     endp


; Timer Interrupt (To close garage door) (int 43h)
timer_isr   proc near
            timer_end		; Reset timer
            mov dl, door
            cmp dl, 0FFh	; Close door if open
            jne int43_end
            motor_wake
            mov ah, 1
            call motor_rot
            motor_sleep
            not dl
            mov door, dl
int43_end:  iret
timer_isr   endp

; Constants in ROM
empty_msg   db      'EMPTY'
full_msg    db      'FULL'

; To skip to RAM
            db      1000h dup(?)

; Variables            
cars        dw      0           ; Number of cars in garage
cars_dec    db      '0000'      ; Number of cars in decimal form
sensor1     db      0FFh        ; 00h - IR break, FFh - IR sensed
sensor2     db      0FFh        ; Sensor 1 - entry, Sensor 2 - exit
door        db      00h         ; Garage door: 00h - closed, FFh - open
direction   db      11h         ; Direction of vehicle: 00h - exit, FFh - entry, 11h - null state