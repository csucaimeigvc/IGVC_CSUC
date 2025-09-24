#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x25f8bfc1, "module_layout" },
	{ 0x9783df10, "usb_serial_deregister_drivers" },
	{ 0xfd26958d, "usb_serial_register_drivers" },
	{ 0x37a0cba, "kfree" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xd58d4b67, "kmem_cache_alloc_trace" },
	{ 0xc1c7e6c0, "kmalloc_caches" },
	{ 0x300209fa, "usb_clear_halt" },
	{ 0x856b5bbf, "tty_encode_baud_rate" },
	{ 0x409873e3, "tty_termios_baud_rate" },
	{ 0x6c257ac0, "tty_termios_hw_change" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0xd4bacb8, "usb_kill_urb" },
	{ 0x27a2116c, "usb_control_msg" },
	{ 0x4c1a2642, "tty_flip_buffer_push" },
	{ 0xf3b3b98e, "__tty_insert_flip_char" },
	{ 0x2d504617, "tty_buffer_request_room" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x3ea1b6e4, "__stack_chk_fail" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x1000e51, "schedule" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x89c9a638, "_dev_err" },
	{ 0xab2205bf, "usb_serial_port_softint" },
	{ 0x46834faf, "usb_submit_urb" },
	{ 0x4829a47e, "memcpy" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x1fdc7df2, "_mcount" },
};

MODULE_INFO(depends, "usbserial");

MODULE_ALIAS("usb:v1A86p7523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86p5523d*dc*dsc*dp*ic*isc*ip*in*");
