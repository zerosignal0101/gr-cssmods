# CSS mod and demod OOT module

**Please notice that the project cannot achieve ideal performance.** It is just a testing project (draft level). Because it has 0.1% packet error rate even in simulation circumstance (lol).

A small testing project for Chirp Spread Spectrum (CSS) modulation and demodulation (like LoRa but not the same phy structure)

## Project install

Just clone the project and make a `build` dir.

```shell
cmake ../
make -j6 && sudo make install
sudo ldconfig
```

Then you can see the folder `cssmods` in the gnuradio companion.

## Modules

**CSS Symbols** will transform the PDU with bytes to PDU with symbols (uint32_t).

**CSS Modulate** will modulate symbols to complex stream with constant speed.

**CSS Demodulate Symbols** will do frame sync and demodulate the symbols from the stream. And then output a pdu consists of symbols (uint32_t).

**CSS Symbols Decode** will decode the symbols to PDU with bytes.

That's all! If you find something that can be improved, I am happy to hear from you.
