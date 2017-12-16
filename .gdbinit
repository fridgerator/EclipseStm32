# Example configuration file for using orbuculum with BMP
# ...either a genuine one, or a 'Bluepill' variant

source ~/git/orbuculum/Support/gdbtrace.init
target extended-remote /dev/ttyBmpGdb
monitor swdp_scan
file ~/git/EclipseStm32/Debug/EclipseStm32.elf
attach 1
set mem inaccessible-by-default off
set print pretty
load
start

# Configure STM32F1 SWD pin
enablestm32swd

# ==== This Section for Bluepill =======
# 2.25Mbps, don't use TPIU, don't use Manchester encoding
# ConfigCoreClock is the current speed of the clock which the
# divisors are based on...hence the reason it's needed
# monitor traceswo 2250000
# prepareSWD ConfigCoreClock 2250000 0 0
# ======================================

# ==== This Section for genuine BMP =======
# 200Kbps, don't use TPIU and Manchester encoding
# Typically used for 'real' BMP
monitor traceswo
prepareSWD 24000000 200000 0 1
# =========================================

dwtSamplePC 1
dwtSyncTAP 3
dwtPostTAP 1
dwtPostInit 1
dwtPostReset 15
dwtCycEna 1

ITMId 1
ITMGTSFreq 3
ITMTSPrescale 3
ITMTXEna 1
ITMSYNCEna 1
ITMEna 1

ITMTER 0 0xFFFFFFFF
ITMTPR 0xFFFFFFFF

