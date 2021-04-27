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
port1B      equ     02h         ; Interrupt vector number (i/p)
port1C      equ     04h         ; 0-1, 7: LCD RS, RW, E  (o/p)
port1CR     equ     06h         ; Control register   

; 8255 (2)
port2A      equ     08h         ; ADC data lines (i/p)
port2B      equ     0Ah         ; 0, 7: Motor step, dir pins (o/p)
port2C      equ     0Ch         ; 0, 7: ADC RD, WR pins <o/p>
port2CR     equ     0Eh         ; Control register 

            
            jmp st1
            nop
            dd      00000000h
            
            dw      resolve_isr ; nmi - int 2h
            dw      0000h
            
            db      244 dup(0)  ; int 3h - int 3Eh
            
            dw      adc_isr     ; int 40h
            dw      0000h
            
            dw      ir1_isr     ; int 41h
            dw      0000h
            
            dw      ir2_isr     ; int 42h
            dw      0000h
            
            dw      button_isr  ; int 43h
            dw      0000h
            
            db      752 dup(0)
         
             
st1:                 
            cli
            
            mov ax, 0000h       ; Initialising segment registers       
            mov ds, ax
            mov es, ax
            mov ss, ax
            mov sp, 0FFFEh
            mov si, 0000h
        
            mov al, 10000010b   ; Initialising the 8255s I/O ports
            out port1CR, al
            mov al, 10010000b
            out port2CR, al
            call delay_1ms
        
            mov al, 38h         ; Initialising the LCD
            call lcd_cmd
            mov al, 38h
            call lcd_cmd
            mov al, 0Fh
            call lcd_cmd
            mov al, 06h
            call lcd_cmd
            mov al, 01h
            call lcd_cmd
            mov al, 80h
            call lcd_cmd
            mov ax, cars        ; Initial number of cars
            call lcd_write
            
            mov al, 81h         ; High RD and high WR to initialise ADC
            out port2C, al
            
inf_loop:   jmp inf_loop        ; Infinite loop


; Procedure for approximately 5-minute delay (10 seconds because simulation)
delay_5min  proc near
            mov ah, 0FFh
            mov dx, 6000
dec_5min:   call delay_1ms
            cmp byte ptr door, 00h
            jne dec_dx
            mov ah, 00h
            ret
dec_dx:     dec dx
            jnz dec_5min
            not byte ptr door
            ret
delay_5min  endp


; Procedure for approximately 5-second delay
delay_5sec  proc near
            mov dx, 2000
dec_5sec:   call delay_1ms
            dec dx
            jnz dec_5sec
            ret
delay_5sec  endp
              
              
; Procedure for approximately little over 1 ms delay
delay_1ms   proc near
            mov cx, 420        
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
            mov al, 07h
            out port1C, al
            call delay_lcd
            mov al, 05h
            out port1C, al
            call delay_lcd
            mov al, 04h
            out port1C, al
            call delay_lcd
            mov al, bl
            out port1A, al
            mov al, 00h
            out port1C, al
            call delay_lcd
            pop cx
            ret
lcd_cmd     endp


; Procedure to send data to LCD, AL - data        
lcd_data    proc near
            push cx
            mov bl, al
            mov al, 07h
            out port1C, al
            call delay_lcd
            mov al, 05h
            out port1C, al
            call delay_lcd
            mov al, 05h
            out port1C, al
            call delay_lcd
            mov al, bl
            out port1A, al
            mov al, 01h
            out port1C, al
            call delay_lcd
            pop cx
            ret
lcd_data    endp


; Procedure to write a number (or string if empty or full) to LCD, AX - number
lcd_write   proc near
            mov di, ax
            mov al, 01h       ; Clear screen
            call lcd_cmd
            mov al, 81h
            call lcd_cmd
            lea si, empty_msg
            mov cl, 5
            cmp di, 0
            je empty_lcd
            lea si, full_msg
            mov cl, 4
            cmp di, 2000
            je full_lcd
            
            lea si, cars_dec
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
            
            lea si, cars_dec
            mov cl, 4
num_lcd:    lodsb
            call lcd_data
            dec cl
            jnz num_lcd
            jmp wr_end
            
empty_lcd:  lodsb
            call lcd_data
            dec cl
            jnz empty_lcd
            jmp wr_end
            
full_lcd:   lodsb
            call lcd_data
            dec cl
            jnz full_lcd
            
wr_end:     ret
lcd_write   endp


; Procedure for one full rotation of motor opening; AH = 0 (open/clockwise), 1 (close, counterclockwise); 
motor_rot   proc near
            mov bl, 4           ; 4 steps for 1 rotation (360 / 90) 
            cmp ah, 0           ; set direction
            jne close
            mov al, 00h         ; DIR 0 for CW
            out port2B, al
rot_open:   call delay_5sec
            mov al, 00h         ; step pulse low
            out port2B, al
            call delay_1ms
            mov al, 01h         ; step pulse high
            out port2B, al
            dec bl
            jnz rot_open
            jmp rot_end
            
close:      mov al, 80h         ; DIR 1 for CCW
            out port2B, al
rot_close:  call delay_5sec
            mov al, 80h         ; step pulse low
            out port2B, al
            call delay_1ms
            mov al, 81h         ; step pulse high
            out port2B, al
            dec bl
            jnz rot_close    
rot_end:    ret
motor_rot   endp


; Procedure to begin AD conversion
adc_begin   proc near
            mov al, 01h         ; WR' low
            out port2C, al
            call delay_1ms
            int 40h
            ret
adc_begin   endp


; Interrupt Resolver (int 2h - NMI)
resolve_isr proc near
            in al, port1B
            mov bl, al
            xor bh, bh
            mov cl, 2
            shl bx, cl
            mov dx, [bx]
            pushf
            push cs
            call dx
            iret
resolve_isr endp


; ADC Interrupt (int 40h)
adc_isr     proc near
            mov al, 81h         ; WR' high
            out port2C, al
            call delay_1ms
            mov al, 80h         ; RD' low
            out port2C, al
            in al, port2A
            cmp al, 0Bh         ; 11/255 = approx 0.04, 0.04 * 5000kg = 200kg - lower weight limit of car for garage
            jb nmi_end
            
            mov dl, direction
            cmp dl, 00h
            jne chk_entry
            mov ax, cars
            cmp ax, 0
            je nmi_end
            dec ax
            mov cars, ax
            call lcd_write
            jmp nmi_end
            
chk_entry:  cmp dl, 0FFh
            jne nmi_end
            mov ax, cars
            cmp ax, 2000
            je nmi_end
            inc ax
            mov cars, ax
            call lcd_write
            
nmi_end:    mov al, 81h         ; RD' high, WR' high
            out port2C, al
            mov dl, 11h         ; Resetting direction to null
            mov direction, dl
            mov byte ptr sensor1, 0FFh
            mov byte ptr sensor2, 0FFh                        
            iret
adc_isr     endp


; IR Sensor 1 Interrupt (int 41h)
ir1_isr     proc near
            mov dl, sensor1
            not dl		        ; Update sensor status to IR line broken
            mov sensor1, dl
            mov dl, sensor2
            cmp dl, 00h
            jne int41_end	
            mov dl, 00h		    ; Set direction variable if both sensors have been triggered
            mov direction, dl
            call adc_begin      ; Start AD conversion if car has triggered other sensor
int41_end:  iret
ir1_isr     endp


; IR Sensor 2 Interrupt (int 42h)
ir2_isr     proc near
            mov dl, sensor2
            not dl
            mov sensor2, dl
            mov dl, sensor1
            cmp dl, 00h
            jne int42_end	
            mov dl, 0FFh		; Set direction variable if both sensors have been triggered
            mov direction, dl
            call adc_begin      ; Start AD conversion if car has triggered other sensor
int42_end:  iret
ir2_isr     endp


; Button Interrupt (To open/close garage door) (int 43h)
button_isr  proc near
            mov dl, door
            not byte ptr door
            cmp dl, 00h
            jne int40_cls
            mov ah, 0
            call motor_rot
            call delay_5min
            cmp ah, 00h
            je int40_end                                          
            
int40_cls:  mov ah, 1
            call motor_rot
            
int40_end:  iret
button_isr  endp 

; Constants in ROM
empty_msg   db      'EMPTY'
full_msg    db      'FULL'

; Skip to RAM
            db      1c00h dup(?)
            
; Variables in RAM
cars        dw      0000        ; Number of cars in garage
cars_dec    db      '0000'      ; Number of cars in decimal form
sensor1     db      0FFh        ; 0 - IR break, FFh - IR sensed
sensor2     db      0FFh        ; Sensor 1 - entry, Sensor 2 - exit
door        db      00h         ; Garage door: 00h - closed, FFh - open
direction   db      11h         ; Direction of vehicle: 00h - exit, FFh - entry, 11h - null state