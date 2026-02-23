input = """#define KEY_BACKSPACE   0x08
#define KEY_TAB         0x09
#define KEY_ENTER       0x0A
// 0x0D - CARRIAGE RETURN
#define KEY_BTN_LEFT2   0x11
#define KEY_BTN_RIGHT2  0x12


#define KEY_MOD_ALT     0xA1
#define KEY_MOD_SHL     0xA2
#define KEY_MOD_SHR     0xA3
#define KEY_MOD_SYM     0xA4
#define KEY_MOD_CTRL    0xA5

#define KEY_ESC       0xB1
#define KEY_UP        0xb5
#define KEY_DOWN      0xb6
#define KEY_LEFT      0xb4
#define KEY_RIGHT     0xb7

#define KEY_BREAK     0xd0
#define KEY_INSERT    0xD1
#define KEY_HOME      0xD2
#define KEY_DEL       0xD4
#define KEY_END       0xD5
#define KEY_PAGE_UP    0xd6
#define KEY_PAGE_DOWN  0xd7

#define KEY_CAPS_LOCK   0xC1

#define KEY_F1 0x81
#define KEY_F2 0x82
#define KEY_F3 0x83
#define KEY_F4 0x84
#define KEY_F5 0x85
#define KEY_F6 0x86
#define KEY_F7 0x87
#define KEY_F8 0x88
#define KEY_F9 0x89
#define KEY_F10 0x90
"""

for line in input.splitlines():
    if line.startswith("#"):
        elm = line.split(" ")
        print(f"key_codes::{elm[1]} => {{}}")
