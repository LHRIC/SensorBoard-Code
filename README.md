# SensorBoard-Code

## To get VSCode Intellisense

Put the below JSON snippet in your .vscode directory (create if you don't have)
as c_cpp_properties.json
```json
{
    "configurations": [
        {
            "name": "STM32CubeIDE",
            "includePath": [
                "${workspaceFolder}/Core/Inc",
                "${workspaceFolder}/Drivers/STM32F0xx_HAL_Driver/Inc",
                "${workspaceFolder}/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy",
                "${workspaceFolder}/Drivers/CMSIS/Device/ST/STM32F0xx/Include",
                "${workspaceFolder}/Drivers/CMSIS/Include"
            ],
            "defines": [
                "DEBUG",
                "USE_HAL_DRIVER",
                "STM32F042x6"
            ],
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "gcc-arm"
        }
    ],
    "version": 4
}
```

This works for all STM32CubeIDE generated projects, but you might need to customize the defines and include paths depending on the processor used. You can see the used include paths in the STM32CubeIDE under:

```Project > Properties > C/C++ Build > Settings > MCU GCC Compiler > Include Paths```

and for defines 

```Project > Properties > C/C++ Build > Settings > MCU GCC Compiler > Preprocessor```
