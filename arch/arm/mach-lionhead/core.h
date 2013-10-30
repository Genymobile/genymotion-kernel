/* 2MB large area for motherboard's peripherals static mapping */
#define LIONHEAD_PERIPH 0xf8000000

extern struct smp_operations	lionhead_smp_ops;

extern void lionhead_cpu_die(unsigned int cpu);
