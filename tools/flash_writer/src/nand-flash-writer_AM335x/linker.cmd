-l rtsv5_A_le_eabi.lib
-stack          0x00004000 /* Stack Size */
-heap           0x00002000 /* Heap Size */

MEMORY
{
  RAM0           org=0x402F8000 len=0x00017fff /* OCM RAM0 */
  //RAM          org=0x40200000 len=0x0000FCB0 /*RAM*/
  //DRAM         org=0x80000000 len=0x00200000 /* DDR for Memory Allocation */
  DRAM           org=0x40400000 len=0x00020000 /*RAM*/
 }

SECTIONS
{
  .text :
  {
  } > RAM0
  .const :
  {
  } > RAM0
  .bss :
  {
  } > RAM0
  .far :
  {
  } > RAM0
  .stack :
  {
  } > RAM0
  .data :
  {
  } > RAM0
  .cinit :
  {
  } > RAM0
  .sysmem :
  {
  } > RAM0
  .cio :
  {
  } > RAM0
  .switch :
  {
  } > RAM0
  
 .ddrram :
  {
  } > DRAM

}
  



