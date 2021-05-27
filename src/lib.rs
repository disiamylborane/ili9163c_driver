/*! 

The middleware for type-safe access to ILI9163C-based LCD displays.
 
The library wraps around a user-provided low-level protocol descriptor.

# Usage steps
1. Create a custom type for SPI or 8080/6800 bus representation.
2. Implement [Ili9163CInterface](trait.Ili9163CInterfaceCtx.html) or 
[Ili9163CInterfaceCtx](trait.Ili9163CInterfaceCtx.html) for a custom type
3. Create the [Ili9163C<T: Ili9163CInterface>](type.Ili9163C.html)
or [Ili9163CCtx<T: Ili9163CInterfaceCtx>](struct.Ili9163CCtx.html) around the custom type
4. Use the methods of Ili9163C to initialize and use the LCD

# Example

```
// Assume the device is connected through 8080 parallel interface and accessed globally.
// No context pointer is needed when sending and receiving the data

// Describe the low layer protocol
struct MyLcd;
impl crate::Ili9163CInterface for MyLcd {
    unsafe fn send_write_cmd(command: u8) {
        bus8080::send_command(command)
    }
    unsafe fn send_read_cmd(command: u8) {
        // Reading through 8080/6800 bus requires a dummy read cycle
        bus8080::send_command(command);
        let _unused = bus8080::read_data();
    }
    unsafe fn data_write(data: u8) {
        bus8080::send_data(command);
    }
    unsafe fn data_read() -> u8 {
        lcd::read_data()
    }
}

// Use the driver
{
    ...
    let lcd = Ili9163C::<MyLcd>::new();

    // Initialize the driver
    lcd.sleep_off_nonsynced();  // Exit sleep (sleep mode is activated after reset)
    delay_ms(5);
    lcd.set_fps_normal(r!([0 63] 40), r!([4 35] 20));  // Set refresh rate parameters
    lcd.set_lcd_inversion(LCDInversion::Line, LCDInversion::Line, LCDInversion::Line);  // Set polarity inversion type
    lcd.set_mem_access(MemAccess(  // Setup access parameters
        RowAddressOrder::RightToLeft,
        ColumnAddressOrder::BottomToTop, 
        RowColumnOrder::RowByRow,
        LineAddressOrder::RefreshTopDown,
        ColourOrder::BGR,
        DisplayDataLatchOrder::LeftToRight,
    ));
    lcd.set_pixel_format(ControlPixFormat::Bits_16, /*don't care*/RGBPixFormat::Bits_16);  // Setup data representation
    lcd.display_on();  // Enable display output (disabled after reset)
    lcd.mode_normal();  // Enter normal mode (Out of Partial, Scroll mode)


    // Now draw a rectangular smiley :)

    lcd.set_window(0..128, 0..128);    // Clear screen
    lcd.write_n_times_16(0, 128*128);  // Assume the LCD resolution is 128x128.

    lcd.set_window(20..30, 20..30);      // Left(right?) eye
    lcd.write_n_times_16(0xFFE0, 10*10); // Fill yellow

    lcd.set_window(50..60, 20..30);      // Right(Left?) eye
    lcd.write_n_times_16(0xFFE0, 10*10); // Fill yellow

    lcd.set_window(20..60, 50..55);      // Mouth
    lcd.write_n_times_16(0xFFE0, 40*5);  // Fill yellow
    ...
}
```
*/

#![no_std]

#![warn(missing_docs)]

#![allow(incomplete_features)]
#![feature(const_generics)]
#![feature(const_evaluatable_checked)]
#![feature(const_panic)]

#![feature(doc_cfg)]

extern crate ranged_integers;
use ranged_integers::*;

extern crate bool_enum;
use bool_enum::bool_enum;

use core::ops::Range;
use core::marker::PhantomData;


/// Device: The descriptor for ILI9163C interface to be implemented
/// 
/// The trait supplies the read and write functions used by the library. The functions' content
/// is related to the communication protocol.
/// 
/// When the *use_context* feature is selected, the trait functions are methods
/// accepting `&mut self`.
/// 
/// When the *use_no_context* feature is selected, the trait functions are just the 
/// static functions, they don't accept self references.
pub trait Ili9163CInterfaceCtx {
    /// Send the command followed by write data operation (or just the command without parameters).
    /// 
    /// For Serial interface: perform a single write operation, D/C bit = 0
    /// 
    /// For 8080/6800 interface: perform a single write operation
    unsafe fn send_write_cmd(&mut self, command: u8);

    /// Send the command followed by read data operation.
    /// 
    /// For Serial interface: perform a single write operation (D/C = 0),
    /// then switch data bus to Hi-Z for reception
    /// 
    /// For 8080/6800 interface: perform a write operation and a dummy read operation
    unsafe fn send_read_cmd(&mut self, command: u8);

    /// Write the data.
    /// 
    /// For Serial interface: perform a write operation with D/C bit = 1
    /// 
    /// For 8080/6800 interface: perform a write operation with D/C bit = 1
    unsafe fn data_write(&mut self, data: u8);

    /// Read the data.
    /// 
    /// For Serial interface: perform a read operation with D/C bit = 1
    /// 
    /// For 8080/6800 interface: perform a read operation with D/C bit = 1
    unsafe fn data_read(&mut self) -> u8;
}

/// Device: The context-free version of [Ili9163CInterfaceCtx](trait.Ili9163CInterfaceCtx.html)
#[allow(missing_docs)]
pub trait Ili9163CInterface
{
    unsafe fn send_write_cmd(command: u8);
    unsafe fn send_read_cmd(command: u8);
    unsafe fn data_write(data: u8);
    unsafe fn data_read() -> u8;
}

/// Driver: The type-safe interface to ILI9163C controller
pub struct Ili9163CCtx<T: Ili9163CInterfaceCtx>(pub T);

#[doc(hidden)]
pub struct CtxWrap<T: Ili9163CInterface>(PhantomData<T>);

/// Driver: The type-safe interface to ILI9163C controller
/// for context-free device interface
pub type Ili9163C<T> = Ili9163CCtx<CtxWrap<T>>;


impl<T: Ili9163CInterface> Ili9163CInterfaceCtx for CtxWrap<T> {
    unsafe fn send_write_cmd(&mut self, command: u8){
        T::send_write_cmd(command)
    }
    unsafe fn send_read_cmd(&mut self, command: u8){
        T::send_read_cmd(command)
    }
    unsafe fn data_write(&mut self, data: u8){
        T::data_write(data)
    }
    unsafe fn data_read(&mut self) -> u8{
        T::data_read()
    }
}


/// # ILI9163C creation and initialization
impl<T: Ili9163CInterfaceCtx> Ili9163CCtx<T> {
    /// Create a new instance of ILI9163C
    pub fn with_ctx(t: T) -> Self {
        Self(t)
    }
}

/// # ILI9163C context-free creation and initialization
impl<T: Ili9163CInterface> Ili9163CCtx<CtxWrap<T>> {
    /// Create a new instance of ILI9163C for context-free device interface
    pub fn no_ctx() -> Self {
        Self(CtxWrap(PhantomData::<T>))
    }
}

impl<T: Ili9163CInterfaceCtx> Ili9163CCtx<T> {
    fn send_write_cmd(&mut self, cmd: Command) {
        unsafe {self.0.send_write_cmd(cmd as u8)}
    }
    fn send_read_cmd(&mut self, cmd: Command) {
        unsafe {self.0.send_read_cmd(cmd as u8)}
    }
    fn data_write(&mut self, data: u8) {
        unsafe {self.0.data_write(data)}
    }
    fn data_read(&mut self) -> u8 {
        unsafe {self.0.data_read()}
    }

    fn data_write16(&mut self, val: u16)  {
        let vals = val.to_be_bytes();
        self.data_write(vals[0]);
        self.data_write(vals[1]);
    }

    fn data_read16(&mut self) -> u16 {
        let hi = self.data_read();
        let lo = self.data_read();
        u16::from_be_bytes([hi, lo])
    }
}

/// # NOP
impl<T: Ili9163CInterfaceCtx> Ili9163CCtx<T> {
    /// Perform *dummy* command.
    /// Use this to stop RAM read and write operatinos
    pub fn no_operation(&mut self) {
        self.send_write_cmd(Command::Nop);
    }
}



/// # LCD display ID
impl<T: Ili9163CInterfaceCtx> Ili9163CCtx<T> {
    /// Read all the Display_ID bytes. Usually reads 0x54,0x80,0x66
    pub fn read_display_identification_info(&mut self) -> DisplayIDInfo {
        self.send_read_cmd(Command::ReadIDAll);
        let manufacture_id = self.data_read();
        let module_id_1 = self.data_read();
        let module_id_2 = self.data_read();
        DisplayIDInfo{manufacture_id, module_id_1, module_id_2}
    }

    /// Read the Driver ID4 bytes (ID1..3 are removed in ILI9163C).
    pub fn read_display_id4(&mut self) -> DriverIDInfo {
        self.send_read_cmd(Command::ReadID4);
        let driver_ic_id = self.data_read();
        let driver_ic_part_number = self.data_read();
        let driver_ic_version = unsafe {Ranged::__unsafe__((self.data_read() % 16) as u8)} ;
        DriverIDInfo{driver_ic_id, driver_ic_part_number, driver_ic_version}
    }

    /// Read the Display_ID first bytes (Usually 0x54)
    pub fn read_display_manufacture_id(&mut self) -> u8 {
        self.send_read_cmd(Command::ReadID1);
        self.data_read()
    }

    /// Read the Display_ID second byte (Usually 0x80)
    pub fn read_display_module_id_1(&mut self) -> u8 {
        self.send_read_cmd(Command::ReadID2);
        self.data_read()
    }

    /// Read the Display_ID third byte (Usually 0x66)
    pub fn read_display_module_id_2(&mut self) -> u8 {
        self.send_read_cmd(Command::ReadID3);
        self.data_read()
    }

}


/// Definition: The complete display status
#[allow(missing_docs)]
pub struct DisplayStatus {
    pub powermode : PowerMode,
    pub mem_access: MemAccess,
    pub pixfmt : ControlPixFormat,
    pub negative: bool,
    pub vertical_scroll: bool,
    pub gc: GammaCurve,
    pub tearing: TearingPinMode,
}

/// # Display status
impl<T: Ili9163CInterfaceCtx> Ili9163CCtx<T> {

    /// Read the complete display status
    pub fn display_status(&mut self) -> Result<DisplayStatus,()> {
        self.send_read_cmd(Command::DisplayStatus);
        let byte0 = self.data_read();
        let byte1 = self.data_read();
        let byte2 = self.data_read();
        let byte3 = self.data_read();

        let powermode = PowerMode{
            booster_on: byte0.bit(7),
            display_on: byte2.bit(2),
            idle: byte1.bit(3),
            normal: byte1.bit(0),
            partial: byte1.bit(2),
            sleep: byte1.bit(1)
        };
        
        let mem_access = MemAccess(
            byte0.bit(6).into(),
            byte0.bit(5).into(),
            byte0.bit(4).into(),
            byte0.bit(3).into(),
            byte0.bit(2).into(),
            byte0.bit(1).into(),
        );

        let pixfmt = match (byte1>>4) & 0b111 {
            0b_011 => ControlPixFormat::Bits_12,
            0b_101 => ControlPixFormat::Bits_16,
            0b_110 => ControlPixFormat::Bits_18,
            _ => Err(())?,
        };

        let negative = byte2.bit(5);
        let vertical_scroll = byte2.bit(7);
        let gc = match byte3>>6 {
            0b_00 => GammaCurve::GC1,
            0b_01 => GammaCurve::GC2,
            0b_10 => GammaCurve::GC3,
            0b_11 => GammaCurve::GC4,
            _ => Err(())?,
        };

        let tearing = 
            if byte2.bit(1) {
                if byte3.bit(5) { TearingPinMode::VBlankHBlank }
                else { TearingPinMode::VBlankOnly }
            }
            else { TearingPinMode::Disabled };

        Ok(DisplayStatus{
            powermode,
            mem_access,
            pixfmt,
            negative,
            vertical_scroll,
            gc,
            tearing
        })
    }

    /// Read the current power mode
    pub fn read_power_mode(&mut self) -> PowerMode {
        self.send_read_cmd(Command::ReadDisplayPowerMode);
        let mode = self.data_read();
        PowerMode {
            booster_on: mode.bit(7),
            idle: mode.bit(6),
            partial: mode.bit(5),
            sleep: mode.bit(4),
            normal: mode.bit(3),
            display_on: mode.bit(2),
        }
    }

}

/// # LCD interface parameters
impl<T: Ili9163CInterfaceCtx> Ili9163CCtx<T> {
    /// Read the current memory access mode
    pub fn read_mem_access(&mut self) -> MemAccess {
        self.send_read_cmd(Command::ReadDisplayMemAccessControl);
        let mode = self.data_read();
        MemAccess (
            mode.bit(7).into(),
            mode.bit(6).into(),
            mode.bit(5).into(),
            mode.bit(4).into(),
            mode.bit(3).into(),
            mode.bit(2).into(),
        )
    }

    /// Read the current pixel format
    pub fn get_pixel_format(&mut self) -> (Result<ControlPixFormat, ()>, Result<RGBPixFormat, ()>) {
        self.send_read_cmd(Command::ReadDisplayPixelFormat);
        let mode = self.data_read();
        (
            match mode & 0b111 {
                0b_011 => Ok(ControlPixFormat::Bits_12),
                0b_101 => Ok(ControlPixFormat::Bits_16),
                0b_110 => Ok(ControlPixFormat::Bits_18),
                _ => Err(()),
            },
            match (mode>>4) & 0b1111  {
                0b_0101 => Ok(RGBPixFormat::Bits_16),
                0b_0110 => Ok(RGBPixFormat::Bits_18_OneTransfer),
                0b_1110 => Ok(RGBPixFormat::Bits_18_ThreeTransfers),
                _ => Err(()),
            }
        )
    }

    /// Read the current image mode
    pub fn read_image_mode(&mut self) -> (ImageMode, Result<GammaCurve, ()>) {
        self.send_read_cmd(Command::ReadDisplayImageMode);
        let mode = self.data_read();
        (
            ImageMode {
                vert_scrolling_on: mode.bit(7),
                horz_scrolling_on: mode.bit(6),
                negative_on: mode.bit(5),
                all_pixels_on: mode.bit(4),
                all_pixels_off: mode.bit(3),
            },

            match mode & 0b111  {
                0 => Ok(GammaCurve::GC1),
                1 => Ok(GammaCurve::GC2),
                2 => Ok(GammaCurve::GC3),
                3 => Ok(GammaCurve::GC4),
                _ => Err(()),
            }
        )
    }

    /// Read the current internal signal mode
    pub fn read_signal_mode(&mut self) -> SignalMode {
        self.send_read_cmd(Command::ReadDisplaySignalMode1);
        let mode = self.data_read();
        SignalMode {
            tearing: if mode.bit(7) {
                        if mode.bit(6) {TearingPinMode::VBlankHBlank}
                        else {TearingPinMode::VBlankOnly}
                    } else {TearingPinMode::Disabled},
            horz_sync_on: mode.bit(5),
            vert_sync_on: mode.bit(4),
            pixclock_on: mode.bit(3),
            data_enable: mode.bit(2),
        }
    }

    /// Setup the interface pixel format
    pub fn set_pixel_format(&mut self, cfmt: ControlPixFormat, rgbfmt: RGBPixFormat){
        self.send_write_cmd(Command::InterfacePixelFormat);
        self.data_write(
            match cfmt {
                ControlPixFormat::Bits_12 => 0b_011,
                ControlPixFormat::Bits_16 => 0b_101,
                ControlPixFormat::Bits_18 => 0b_110,
            } | 
            (match rgbfmt {
                RGBPixFormat::Bits_16 => 0b_0101,
                RGBPixFormat::Bits_18_OneTransfer => 0b_0110,
                RGBPixFormat::Bits_18_ThreeTransfers => 0b_1110,
            } << 4)
        )
    }

    /// Set the memory access mode
    pub fn set_mem_access(&mut self, acc: MemAccess) {
        self.send_write_cmd(Command::MemAccessControl);
        self.data_write(
            ((acc.0 as u8) << 7) 
            | ((acc.1 as u8) << 6)
            | ((acc.2 as u8) << 5)
            | ((acc.3 as u8) << 4)
            | ((acc.4 as u8) << 3)
            | ((acc.5 as u8) << 4)
        );
    }

    /// Select the Gamma curve for the display
    pub fn set_gamma_curve(&mut self, gc: GammaCurve) {
        self.send_write_cmd(Command::GammaCurveSet);
        self.data_write(match gc{
            GammaCurve::GC1 => 1,
            GammaCurve::GC2 => 2,
            GammaCurve::GC3 => 4,
            GammaCurve::GC4 => 8,
        });
    }
    
    /// Set the gamma adjustment mode
    pub fn set_gamma_adjustment_enabled(&mut self, en: bool) {
        self.send_write_cmd(Command::GammaCorrectionSettingEnabled);
        self.data_write(en as u8);
    }

    /// Enable or disable tearing output signal
    pub fn tearing_setup(&mut self, mode: TearingPinMode) {
        match mode {
            TearingPinMode::Disabled => {self.send_write_cmd(Command::TearingEffectLineOff);}
            TearingPinMode::VBlankOnly => {
                self.send_write_cmd(Command::TearingEffectLineOn);
                self.data_write(0);
            }
            TearingPinMode::VBlankHBlank => {
                self.send_write_cmd(Command::TearingEffectLineOn);
                self.data_write(1);
            }
        }
    }

}



/// # LCD working mode state
impl<T: Ili9163CInterfaceCtx> Ili9163CCtx<T> {
    /// Enter normal mode
    pub fn mode_normal(&mut self) { 
        self.send_write_cmd(Command::NormalModeOn);
    }

    /// Enter partial mode
    pub fn mode_partial(&mut self) {
        self.send_write_cmd(Command::PartialModeOn);
    }

    /// Set the display region for partial mode
    pub fn set_partial_area(&mut self, yrange: Range<u16>) {
        self.send_write_cmd(Command::SetPartialArea);
        self.data_write16(yrange.start);
        self.data_write16(yrange.end-1);
    }
}


/// # LCD working mode toogles
impl<T: Ili9163CInterfaceCtx> Ili9163CCtx<T> {
    /// Software reset: set all the parameters to their default values
    pub fn reset(&mut self) {
        self.send_write_cmd(Command::SoftwareReset);
    }

    /// Enter the sleep mode
    pub fn sleep_on(&mut self) {
        self.send_write_cmd(Command::SleepOn);
    }

    /// Send 'exit sleep mode' command. Wait for 5 ms after
    pub fn sleep_off_nonsynced(&mut self) {
        self.send_write_cmd(Command::SleepOff);
    }

    /// Enter reduced-colour mode: low power, 8 colours
    pub fn idle_on(&mut self) { self.send_write_cmd(Command::IdleOn); }
    /// Exit reduced-colour mode
    pub fn idle_off(&mut self) { self.send_write_cmd(Command::IdleOff); }

    /// Inverse the colour's on the display
    pub fn negative_on(&mut self) { self.send_write_cmd(Command::NegativeOn); }
    /// Stop the colour inversion on the display
    pub fn negative_off(&mut self) { self.send_write_cmd(Command::NegativeOff); }

    /// Recover from display_off mode
    pub fn display_on(&mut self) { self.send_write_cmd(Command::DisplayOn); }
    /// Enter display_off mode, show the blank page
    pub fn display_off(&mut self) { self.send_write_cmd(Command::DisplayOff); }

}


/// # LCD RAM input and output
impl<T: Ili9163CInterfaceCtx> Ili9163CCtx<T> {
    /// Define the area to write the pixel data to
    pub fn set_window(&mut self, x: Range<u16>, y: Range<u16>) {
        self.send_write_cmd(Command::ColumnActiveWindow);
        self.data_write16(x.start);
        self.data_write16(x.end-1);
        self.send_write_cmd(Command::RowActiveWindow);
        self.data_write16(y.start);
        self.data_write16(y.end-1);
    }

    /// Write command + repeated u8 values
    pub fn write_n_times_16(&mut self, data: u16, count: usize) {
        self.send_write_cmd(Command::WriteRAM);
        for _ in 0..count {
            self.data_write16(data)
        }
    }

    /// Write command + repeated u16 values
    pub fn write_n_times_8(&mut self, data: u16, count: usize) {
        self.send_write_cmd(Command::WriteRAM);
        for _ in 0..count {
            self.data_write16(data)
        }
    }

    /// Send "data write" command and get prepared for data transfer
    /// 
    /// ```
    /// {
    ///     // lcd is exclusively borrowed
    ///     let writer = lcd.initiate_write();
    ///     for d in data {
    ///         writer.send_data_16bit(d);
    ///     }
    ///     // writer is dropped, "no_operation" command is sent
    /// }
    /// ```
    pub fn initiate_write<'a>(&'a mut self) -> DisplayWriter<'a, T> {
        self.send_write_cmd(Command::WriteRAM);
        DisplayWriter(self)
    }

    /// Send "data read" command and get prepared for data transfer
    /// 
    /// ```
    /// {
    ///     // lcd is exclusively borrowed
    ///     let reader = lcd.initiate_read();
    ///     for d in &mut data {
    ///         *d = reader.read_data_16bit();
    ///     }
    ///     // reader is dropped, "no_operation" command is sent
    /// }
    /// ```
    pub fn initiate_read<'a>(&'a mut self) -> DisplayReader<'a, T> {
        self.send_write_cmd(Command::ReadRAM);
        DisplayReader(self)
    }
}

/// Helper: ILI9163C data writer
pub struct DisplayWriter<'a, T: Ili9163CInterfaceCtx>(&'a mut Ili9163CCtx<T>);

impl<'a, T: Ili9163CInterfaceCtx> DisplayWriter<'a,T> {
    /// Transfer a single 8-bit piece
    pub fn send_data_8bit(&mut self, x: u8) {
        self.0.data_write(x);
    }
    /// Transfer a single 16-bit piece
    pub fn send_data_16bit(&mut self, x: u16) {
        self.0.data_write16(x);
    }
}

impl<'a,T: Ili9163CInterfaceCtx> Drop for DisplayWriter<'a,T> {
    fn drop(&mut self){
        self.0.no_operation();
    }
}

/// Helper: ILI9163C data reader
pub struct DisplayReader<'a, T: Ili9163CInterfaceCtx>(&'a mut Ili9163CCtx<T>);

impl<'a, T: Ili9163CInterfaceCtx> DisplayReader<'a,T> {
    /// Read a single 8-bit piece
    pub fn read_data_8bit(&mut self) -> u8 {
        self.0.data_read()
    }
    /// Read a single 16-bit piece
    pub fn read_data_16bit(&mut self) -> u16 {
        self.0.data_read16()
    }
}

impl<'a,T: Ili9163CInterfaceCtx> Drop for DisplayReader<'a,T> {
    fn drop(&mut self){
        self.0.no_operation();
    }
}

/// # FPS and inversion control
impl<T: Ili9163CInterfaceCtx> Ili9163CCtx<T> {
    /// Frame rate control for Normal mode
    /// 
    /// `FPS = 200'000 / [(Line + v) * div]`,
    /// 
    /// * `v in [0, 63]; div in [4, 35]`
    /// 
    /// If display is 128x160,  `Line=160`
    pub fn set_fps_normal(&mut self, v: Ranged<0,63>, div: Ranged<4,35>) {
        self.send_write_cmd(Command::FrameRateControlNormal);
        self.data_write(div.u8()-4);
        self.data_write(v.u8());
    }

    /// Frame rate control for Partial mode (See [set_fps_normal](struct.Ili9163C.html#method.set_fps_normal))
    pub fn set_fps_partial(&mut self, v: Ranged<0,63>, div: Ranged<4,35>) {
        self.send_write_cmd(Command::FrameRateControlPartial);
        self.data_write(div.u8()-4);
        self.data_write(v.u8());
    }

    /// Frame rate control for Idle mode
    pub fn set_fps_idle(&mut self, v: Ranged<0,63>, div: Ranged<4,35>) {
        self.send_write_cmd(Command::FrameRateControlIdle);
        self.data_write(div.u8()-4);
        self.data_write(v.u8());
    }

    /// Polarity inversion mode for LC layers
    pub fn set_lcd_inversion(&mut self, normal: LCDInversion, partial: LCDInversion, idle: LCDInversion) {
        self.send_write_cmd(Command::DisplayInversionControl);
        self.data_write(
            (normal as u8) | ((partial as u8) << 1) | ((idle as u8) << 2)
        );
    }
}

/// Definition: The info reported by Command 0x04.
pub struct DisplayIDInfo {
    /// LCD module’s manufacture ID. Usually 0x54.
    pub manufacture_id: u8,
    /// LCD module/driver version ID 1. Usually 0x80.
    pub module_id_1: u8,
    /// LCD module/driver version ID 2. Usually 0x66.
    pub module_id_2: u8
}
/// Definition: The info reported by Command 0xD3.
pub struct DriverIDInfo {
    /// Driver IC ID code. Usually 0x91.
    pub driver_ic_id: u8,
    /// Driver IC Part number ID. Usually 0x63.
    pub driver_ic_part_number: u8,
    /// Driver IC version ID. Usually 0.
    pub driver_ic_version: Ranged<0,15>
}

trait Bit { fn bit(self, bit: u8) -> bool; }
impl Bit for u8 {fn bit(self, bit: u8) -> bool {self & (1<<bit) != 0}}

/// Definition: ILI9163C display's working mode
pub struct PowerMode {
    /// Booster voltage status
    pub booster_on: bool,
    /// Idle mode activated
    pub idle: bool,
    /// Partial mode activated
    pub partial: bool,
    /// Sleep mode activated
    pub sleep: bool,
    /// Normal mode activated
    pub normal: bool,
    /// Display is on
    pub display_on: bool,
}

#[bool_enum]
/** Toogler: Row fill order */
pub enum RowAddressOrder{LeftToRight = 0, RightToLeft = 1}
#[bool_enum]
/** Toogler: Row fill order */
pub enum ColumnAddressOrder{TopToBottom=0, BottomToTop=1}
#[bool_enum]
/** Toogler: Row fill order */
pub enum RowColumnOrder{RowByRow=0, ColByCol=1}
#[bool_enum]
/** Toogler: Row fill order */
pub enum LineAddressOrder{RefreshTopDown=0, RefreshBottomUp=1}
#[bool_enum]
/** Toogler: Row fill order */
pub enum ColourOrder{RGB=0, BGR=1}
#[bool_enum]
/** Toogler: Row fill order */
pub enum DisplayDataLatchOrder{LeftToRight=0, RightToLeft=1}


/// Definition: Memory access parameters
pub struct MemAccess(
    pub RowAddressOrder, 
    pub ColumnAddressOrder, 
    pub RowColumnOrder, 
    pub LineAddressOrder, 
    pub ColourOrder, 
    pub DisplayDataLatchOrder, 
);


/// Option: Parallel bus pixel data width
#[allow(non_camel_case_types)]
#[allow(missing_docs)]
pub enum ControlPixFormat {Bits_12, Bits_16, Bits_18}

/// Option: RGB bus pixel data width
#[allow(non_camel_case_types)]
#[allow(missing_docs)]
pub enum RGBPixFormat {Bits_16, Bits_18_OneTransfer, Bits_18_ThreeTransfers}


/// Option: Gamma compensation curve preset
#[allow(missing_docs)]
pub enum GammaCurve {GC1, GC2, GC3, GC4}

/// Definition: Image mode
pub struct ImageMode {
    /// Vertical scroll mode on
    pub vert_scrolling_on: bool,
    /// Horizontal scroll mode on (disabled in ILI9163C)
    pub horz_scrolling_on: bool,
    /// Color inversion
    pub negative_on: bool,
    /// (disabled in ILI9163C)
    pub all_pixels_on: bool,
    /// (disabled in ILI9163C)
    pub all_pixels_off: bool,
}


/// Definition: ILI controller output pin settings
pub struct SignalMode {
    /// Tearing signal output mode
    pub tearing: TearingPinMode,
    /// Horizontal sync enabled
    pub horz_sync_on: bool,
    /// Vertical sync enabled
    pub vert_sync_on: bool,
    /// Pixclock sync enabled
    pub pixclock_on: bool,
    /// Data enable line
    pub data_enable: bool,
}

#[bool_enum]
/** Toogler: LCD polarity inversion mode */
pub enum LCDInversion{Line=0, Frame=1}

/// Tearing signal line working mode
pub enum TearingPinMode {
    /// Tearing signal disabled
    Disabled,
    /// Tearing signal serves V-Blanking information
    VBlankOnly,
    /// Tearing signal serves both V-Blanking and H-Blanking
    VBlankHBlank
}


#[allow(dead_code)]
enum Command
{
    Nop = 0x00,
    SoftwareReset = 0x01,

    /// 24-bit display identification information
    ReadIDAll = 0x04,

    /// 32-bit display status
    DisplayStatus = 0x09,

    ReadDisplayPowerMode = 0x0A,
    ReadDisplayMemAccessControl = 0x0B,
    ReadDisplayPixelFormat = 0x0C,
    ReadDisplayImageMode = 0x0D,
    ReadDisplaySignalMode1 = 0x0E,
    ReadDisplaySignalMode2 = 0x0F,

    SleepOn = 0x10,
    SleepOff = 0x11,

    PartialModeOn = 0x12,
    /** No partial, no scroll */ 
    NormalModeOn = 0x13,

    NegativeOff = 0x20,
    NegativeOn = 0x21,

    GammaCurveSet = 0x26,
    DisplayOff = 0x28,
    DisplayOn = 0x29,

    ColumnActiveWindow = 0x2A,
    RowActiveWindow = 0x2B,
    WriteRAM = 0x2C,

    /// Visible change takes effect next time the Frame Memory is written to
    SetupColourLUT = 0x2D,

    ReadRAM = 0x2E,

    /// Partial rows to be displayed with PTLON
    SetPartialArea = 0x30,

    VerticalScrollingDefinition = 0x33,
    TearingEffectLineOff = 0x34,
    TearingEffectLineOn = 0x35,

    MemAccessControl = 0x36,

    VerticalScrollingStartAddress = 0x37,

    IdleOff = 0x38,
    /// 8 colours, low power
    IdleOn = 0x39,

    InterfacePixelFormat = 0x3A,

    FrameRateControlNormal = 0xB1,
    FrameRateControlIdle = 0xB2,
    FrameRateControlPartial = 0xB3,

    DisplayInversionControl = 0xB4,

    RGBInterfaceBlankingPorchSetting = 0xB5, // TODO: RGB-related
    DisplayFunctionSet5 = 0xB6, // TODO: WTH???

    SourceDriverDirectionControl = 0xB7,
    GateDriverDirectionControl = 0xB8,

    PowerControlGVDD = 0xC0,
    PowerControlAVDD = 0xC1,
    PowerControlOpAmpNormal = 0xC2,
    PowerControlOpAmpIdle = 0xC3,
    PowerControlOpAmpPartial = 0xC4,

    VCOMVoltage = 0xC5,
    VCOMOffsetVoltage = 0xC7,

    ReadID4 = 0xD3,

    NvMemoryFunctionController1 = 0xD5,
    NvMemoryFunctionController2 = 0xD6,
    NvMemoryFunctionController3 = 0xD7,

    ReadID1 = 0xDA,  // ILI9163 only
    ReadID2 = 0xDB,  // ILI9163 only
    ReadID3 = 0xDC,  // ILI9163 only

    PositiveGammaCorrectionSetting = 0xE0,
    NegativeGammaCorrectionSetting = 0xE1,
    GammaCorrectionSettingEnabled = 0xF2,
}

