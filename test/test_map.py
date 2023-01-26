"""Test marker map for ADI reference designs"""


def get_test_map():
    # Keys are substrings of test function names (and usually filenames)
    # Any board-fmc name will run a specific tests with key as name

    # NO TESTS YET
    # zynq-zc702-adv7511
    # zynq-zc706-adv7511-fmcadc4
    # zynq-zc706-adv7511-fmcomms11
    # zynq-zc706-adv7511-ad9625-fmcadc2
    # vc707_fmcadc2
    #
    #
    # zynq-zed-adv7511-ad9467-fmc-250ebz
    # zynq-zc706-adv7511-ad9625-fmcadc3
    #
    # zynq-zed-adv7511-cn0363
    # zynq-zc706-adv7511-ad6676-fmc
    #
    # zynq-zc706-adv7511-ad9265-fmc-125ebz
    # zynq-zc706-adv7511-ad9434-fmc-500ebz
    # zynq-zed-adv7511
    # zynq-zc706-adv7511-ad9739a-fmc
    # zynq-zed-imageon
    # zynq-zc706-adv7511-fmcdaq3-revC
    # zynq-zc706-adv7511-fmcjesdadc1
    # zynq-zed-adv7511-fmcmotcon2
    # zynq-zc706-adv7511
    #
    # socfpga_arria10_socdk_cn0506_mii
    # zynq-zc706-adv7511-cn0506-mii
    # zynq-zc706-adv7511-cn0506-rgmii
    # zynq-zc706-adv7511-cn0506-rmii
    # zynq-zed-adv7511-cn0506-mii
    # zynq-zed-adv7511-cn0506-rgmii
    # zynq-zed-adv7511-cn0506-rmii
    # zynqmp-zcu102-rev10-cn0506-mii
    # zynqmp-zcu102-rev10-cn0506-rgmii
    # zynqmp-zcu102-rev10-cn0506-rmii
    #
    # socfpga_cyclone5_de10_nano_cn0540
    # zynq-coraz7s-cn0540
    #
    # socfpga_cyclone5_sockit_arradio
    #
    # zynq-coraz7s-cn0501
    #
    # zynq-zc706-adv7511-ad6676-fmc
    # vc707_ad6676evb
    #
    # zynq-zed-adv7511-cn0577
    #
    # zynqmp-zcu102-rev10-ad9082-m4-l8
    #
    # zynqmp-zcu102-rev10-ad9695
    #
    # zynqmp-zcu102-rev10-ad9783
    #
    # zynqmp-zcu102-rev10-stingray
    # zynqmp-zcu102-rev10-stingray-direct-clk
    # zynqmp-zcu102-rev10-stingray-vcxo100
    # zynqmp-zcu102-rev10-stingray-vcxo100-direct-clk
    #
    # zynq-zed-adv7511-ad7768
    #
    # zynq-zed-adv7511-ad7768-1-evb
    #
    # vcu118_dual_ad9208
    #
    # zynqmp-adrv9009-zu11eg-revb-adrv2crr-fmc-revb-xmicrowave
    # zynqmp-adrv9009-zu11eg-revb-adrv2crr-fmc-revb-fmcbridge

    test_map = {}
    test_map["daq3"] = ["zynqmp-zcu102-rev10-fmcdaq3"]
    test_map["ad9152"] = ["zynqmp-zcu102-rev10-fmcdaq3"]
    test_map["daq2"] = ["zynq-zc706-adv7511-fmcdaq2", "zynqmp-zcu102-rev10-fmcdaq2"]
    test_map["ad9144"] = ["zynq-zc706-adv7511-fmcdaq2", "zynqmp-zcu102-rev10-fmcdaq2"]
    test_map["ad9680"] = ["zynq-zc706-adv7511-fmcdaq2", "zynqmp-zcu102-rev10-fmcdaq2"]
    test_map["adrv9371"] = [
        "zynqmp-zcu102-rev10-adrv9371",
        "zynq-zc706-adv7511-adrv9371",
        "socfpga_arria10_socdk_adrv9371",
    ]
    test_map["adrv9009"] = [
        "socfpga_arria10_socdk_adrv9009",
        "zynqmp-zcu102-rev10-adrv9008-1",
        "zynqmp-zcu102-rev10-adrv9008-2",
        "zynqmp-zcu102-rev10-adrv9009",
    ]
    test_map["fmcomms5"] = [
        "zynqmp-zcu102-rev10-ad9361-fmcomms5",
        "zynq-zc702-adv7511-ad9361-fmcomms5",
        "zynq-zc706-adv7511-ad9361-fmcomms5",
        "zynq-zc706-adv7511-ad9361-fmcomms5-ext-lo-adf5355",
    ]
    test_map["pluto"] = ["pluto"]
    test_map["ad9361"] = [
        "socfpga_cyclone5_sockit_arradio",
        "zynq-zed-adv7511-ad9361-fmcomms2-3",
        "zynq-zc702-adv7511-ad9361-fmcomms2-3",
        "zynq-zc706-adv7511-ad9361-fmcomms2-3",
        "zynqmp-zcu102-rev10-ad9361-fmcomms2-3",
        "zynq-adrv9361-z7035-fmc",
        "zynq-adrv9361-z7035-box",
        "zynq-adrv9361-z7035-bob",
        "zynq-adrv9361-z7035-bob-cmos",
    ]
    test_map["ad9364"] = [
        "socfpga_cyclone5_sockit_arradio",
        "zynq-zc702-adv7511-ad9364-fmcomms4",
        "zynq-zc706-adv7511-ad9364-fmcomms4",
        "zynqmp-zcu102-rev10-ad9364-fmcomms4",
        "zynq-adrv9364-z7020-box",
        "zynq-adrv9364-z7020-bob",
        "zynq-adrv9364-z7020-bob-cmos",
        "zynq-zed-adv7511-ad9364-fmcomms4",
    ]
    test_map["adrv9002"] = [
        "zynq-zc706-adv7511-adrv9002",
        "zynq-zc706-adv7511-adrv9002-rx2tx2",
        "zynq-zed-adv7511-adrv9002",
        "zynq-zed-adv7511-adrv9002-rx2tx2",
        "zynqmp-zcu102-rev10-adrv9002",
        "zynqmp-zcu102-rev10-adrv9002-rx2tx2",
    ]
    test_map["ad9172"] = [
        "zynq-zc706-adv7511-ad9172-fmc-ebz",
        "zynqmp-zcu102-rev10-ad9172-fmc-ebz-mode4",
    ]
    test_map["ad9081"] = [
        "socfpga-arria10-socdk-ad9081",
        "socfpga-arria10-socdk-ad9081-np12",
        "versal-vck190-reva-ad9081",
        "zynq-zc706-adv7511-ad9081",
        "zynqmp-zcu102-rev10-ad9081-204b-txmode9-rxmode4",
        "zynqmp-zcu102-rev10-ad9081-204c-txmode0-rxmode1",
        "zynqmp-zcu102-rev10-ad9081-m8-l4",
        "zynqmp-zcu102-rev10-ad9081-m8-l4-vcxo122p88",
        "zynqmp-zcu102-rev10-ad9081-m4-l8",
    ]
    test_map["fmcomms11"] = ["zynq-zc706-adv7511-fmcomms11"]

    return test_map
