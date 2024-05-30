cmake_minimum_required(VERSION 3.5)
add_library(lib_whill STATIC
        Packet.cpp
        PacketParser.cpp
        PacketReceiver.cpp
        SpeedProfile.cpp
        whill_commands.cpp
        WHILL.cpp
)