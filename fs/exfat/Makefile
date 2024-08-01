# SPDX-License-Identifier: GPL-2.0-or-later
#
# Makefile for the linux exFAT filesystem support.
#

obj-$(CONFIG_EXFAT_FS) += exfat.o

exfat-y	:= blkdev.o  core.o        extent.o  misc.o  super.o \
		   cache.o   core_exfat.o  fatent.o  nls.o   xattr.o
