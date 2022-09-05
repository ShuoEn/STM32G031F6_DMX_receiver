# STM32G031F6_DMX_receiver

- STM32G0 series chip, powerful and low cost
- Compatible with STM32Cube HAL
- Use 1 usart peripheral for DMX, usart2 is for debugging, can be ported to any STM32G0 series easily

This DMX512 receiver project does not check precise DMX512 timings,
only check "SPACE FOR BREAK"(DMX512 starting signal) by dectecting "Frame Error",
so do not use in noisy environment or "BAD" DMX transmitter.


## Build
1. STM32CubeIDE: File-> Open Projects From File System-> Select Directory
2. Open STM32G031F6P6_DMX.ioc: Project-> Generate Code
3. Run-> Run
