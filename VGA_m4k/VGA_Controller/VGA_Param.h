/****************************************************************
 * Skyler Schneider ss868                                       *
 * ECE 5760 Lab 1                                               *
 * 1-Dimensional Cellular Automaton                             *
 * Modified from ECE 5760 DE2 VGA Examples                      *
 ****************************************************************/

// Horizontal Parameter ( Pixel )
parameter H_SYNC_CYC   = 96;  // Original: 95
parameter H_SYNC_BACK  = 48;  // Original: 65
parameter H_SYNC_ACT   = 640; // 640 pixels of content
parameter H_SYNC_FRONT = 16;  // Transferred to back porch to
                              // center screen
                              // Original: 0
parameter H_SYNC_MAX   = 800; // 800 effective pixels (cycles)

// Vertical Parameter ( Line )
parameter V_SYNC_CYC   = 2;   // Original: 2
parameter V_SYNC_BACK  = 32;  // Original: 33
parameter V_SYNC_ACT   = 480; // 480 lines of content
parameter V_SYNC_FRONT = 11;  // Transferred to back porch to
                              // center screen
                              // Original: 10
parameter V_SYNC_MAX   = 525; // 525 effective lines
                              // Original: 525

// Start Offset
parameter X_START      = H_SYNC_CYC + H_SYNC_BACK + 4;
parameter Y_START      = V_SYNC_CYC + V_SYNC_BACK;
