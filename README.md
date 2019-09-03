<div style="text-align:center"><img src="assets/icon.jpg" /></div>
<center><h1>MPU60X0</h1></center>

MPU60X0 is an  STM32 I2C Blocking, Master mode driver for invensense' MPU6050 and MPU6000 IMUs.

Features:
- Blocking mode API
- Takes advantage of burst read and write cycles where possible
- Utilizes bit field packing instead of conventional byte operations to ensure readability
- Ultra lightweight and compact abstractions
- Based on the STM32 HAL firmware.
The API is designed to be conistent and easy to read by utilising struct packing instead of the conventional byte operations.


## Examples
TBD


## Dependencies

STM32Fx HAL I2C driver
STM32Fx HAL UART Driver (Debugging)


## Installation

Copy the header files to your Inc directory





## Known Issues

We utilize bit fields, Clang doesn't raise warnings about the bit field sizes being unable to fit with the byte data struct representations, gcc however raises the warning, This is not a problem at the compilation phase unless you use a flag such as -Werror that points it out as an error.


## Contributing
We employ the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) and  [Nest's embedded c++ styles](https://github.com/openthread/openthread/blob/master/STYLE_GUIDE.md)
However in contrast to [Nest's embedded c++ styles](https://github.com/openthread/openthread/blob/master/STYLE_GUIDE.md), We:
	- utilize modern C++ STL abstractions such as [std::pair](https://en.cppreference.com/w/cpp/utility/pair)
	- [std::chrono::duration](https://en.cppreference.com/w/cpp/chrono/duration) to represent discrete time intervals for timeouts

## License
[MIT License](assets/LICENSE)



