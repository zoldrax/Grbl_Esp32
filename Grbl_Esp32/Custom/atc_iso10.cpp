/*

 based on bdrings test implementation for ATC
 https://github.com/bdring/Grbl_Esp32/blob/ATC_DontUse/Grbl_Esp32/Custom/atc.cpp

 - Added some serial debugging.
 - 

 When the M6 command is received, it will wait until all previous moves have completed, then
 inject normal gcode commands to complete the tool change and return to the point where the
 tool change occured.

 This uses a X position rack. The first position is a tool setter. The other 4 are 
 ATC collets. 
 
 To grab a tool you go above the tool, open the chuck, go down to the
 grab height,, then close the chuck.

 To release a tool you go to the grab height, open the chuck, drop the tool, raise
 to the top and close the checks

 The spindle must not be spinning when the chuck is open or the ATC seals will be destroyed. If 
 the spindle was on,it will turn as soon as the M6 command is received and set a spin down time. 
 It will do some moves to get to the tool.  If the spin down time is not down it will wait, 
 before activating the chuck. Same on spin up. It will wait before releasing control to the file. 

 Each tool will touch off on the tool setter. This saves the Z Mpos of that position
 for each tool.

 If you zero a tool on the work piece, all tools will use the delta determined by the 
 toolsetter to set the tool length offset.

 TODO

 If no tool has been zero'd use the offset from tool #1

*/

const int   TOOL_COUNT     = 5;
const int   ETS_INDEX      = 0;     // electronic tool setter index
const float TOOL_GRAB_TIME = 1.0;  // seconds. How long it takes to grab a tool
#ifndef ATC_MANUAL_CHANGE_TIME
#    define ATC_MANUAL_CHANGE_TIME 5000  // milliseconds ATC is open
#endif

#ifndef ATC_EMPTY_SAFE_HEIGHT
#    define ATC_EMPTY_SAFE_HEIGHT -50.0  // safe X travel over tools while empty
#endif

// Absolute machine position of the toolsetter probe
const float ETS_X = -7.0;
const float ETS_Y = -52.0;
const float ETS_Z = -55.0;

// Absolute machine position of the toolholder rack - tool 1 // Rack Center: -90, Tool Distance 35mm
const float TOOL_RACK_X = -160.0;
const float TOOL_RACK_Y = -0.50;
const float TOOL_RACK_Z = -45.0;

// Offset for each tool (Distance between toolholders)
const float TOOL_OFFSET_X = 35;
const float TOOL_OFFSET_Y = 0.0;
const float TOOL_OFFSET_Z = 0.0;

// Offset for tool loading (ATC move away from toolclip)
const float LOAD_OFFSET_X = 0.0;
const float LOAD_OFFSET_Y = -25.0;
const float LOAD_OFFSET_Z = 0.0;

// Offset for tool release (ATC moves Up)
const float RELEASE_OFFSET_X = 0.0;
const float RELEASE_OFFSET_Y = 0.0;
const float RELEASE_OFFSET_Z = 35.0;


typedef struct {
    float mpos[MAX_N_AXIS];    // the pickup location in machine coords
    float offset[MAX_N_AXIS];  // TLO from the zero'd tool
} tool_t;
tool_t tool[TOOL_COUNT + 1];  // one ETS, plus X tools

float top_of_z;                     // The highest Z position we can move around on
bool  tool_setter_probing = false;  // used to determine if current probe cycle is for the setter
int   zeroed_tool_index   = 1;      // Which tool was zero'd on the work piece

uint8_t current_tool = 0;

bool return_tool(uint8_t tool_num);
bool atc_ETS();
bool set_ATC_open(bool open);
void gc_exec_linef(bool sync_after, const char* format, ...);
bool atc_manual_change();
bool atc_ETS_dustoff();

void machine_init() {
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Machine Init");

    pinMode(ATC_RELEASE_PIN, OUTPUT);
#ifdef ETS_DUST_OFF
    pinMode(ETS_DUST_OFF, OUTPUT);
#endif

    // the tool setter
    tool[ETS_INDEX].mpos[X_AXIS] = ETS_X;
    tool[ETS_INDEX].mpos[Y_AXIS] = ETS_Y;
    tool[ETS_INDEX].mpos[Z_AXIS] = ETS_Z;  // Mpos before collet face triggers probe

    // positions for the tools
    for (size_t i = 1; i <= TOOL_COUNT; i++)
    {
        tool[i].mpos[X_AXIS] = TOOL_RACK_X + ((i-1)*TOOL_OFFSET_X);
        tool[i].mpos[Y_AXIS] = TOOL_RACK_Y + ((i-1)*TOOL_OFFSET_Y);
        tool[i].mpos[Z_AXIS] = TOOL_RACK_Z + ((i-1)*TOOL_OFFSET_Z);
    }

    top_of_z = limitsMaxPosition(Z_AXIS) - homing_pulloff->get();
}

bool atc_tool_change(uint8_t new_tool, bool automatic) {
    bool     spindle_was_on       = false;
    bool     was_incremental_mode = false;  // started in G91 mode
    uint64_t spindle_spin_delay;            // used to make sure spindle has fully spun down and up.
    float    saved_mpos[MAX_N_AXIS] = {};   // the position before the tool change

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Toolchange Request for Tool:%d, old Tool is:%d", new_tool, current_tool);

    if (!automatic) {
        current_tool = new_tool;
        grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "Manual tool change to:%d", current_tool);
        return true;
    }

    if (new_tool == current_tool) {  // if no change, we are done
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC existing tool requested:%d", new_tool);
        return true;
    }

    if (new_tool > TOOL_COUNT) {
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Tool out of range:%d", new_tool);
        return false;
    }

    protocol_buffer_synchronize();                                 // wait for all previous moves to complete
    system_convert_array_steps_to_mpos(saved_mpos, sys_position);  // save current position so we can return to it

    // see if we need to switch out of incremental (G91) mode
    if (gc_state.modal.distance == Distance::Incremental) {
        gc_exec_linef(false, "G90");
        was_incremental_mode = true;
    }

    // is spindle on? Turn it off and determine when the spin down should be done.
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Spindle Down");
    if (gc_state.modal.spindle != SpindleState::Disable) {
        spindle_was_on = true;
        gc_exec_linef(false, "M5");
        spindle_spin_delay = esp_timer_get_time() + (spindle_delay_spindown->get() * 1000.0);  // When will spindle spindown be done.

        // optimize this
        uint64_t current_time = esp_timer_get_time();
        if (current_time < spindle_spin_delay) {
            vTaskDelay(spindle_spin_delay - current_time);
        }
    }

    // ============= Start of tool change ====================

    // Move Z to Top
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t Move Z to Top");
    gc_exec_linef(true, "G53 G0 Z%0.3f", top_of_z);  


    if (!return_tool(current_tool)) {  // does nothing if we have no tool
        // Go on Top of new Tool
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t Go on Top of new Tool Z:Z%0.3f", top_of_z);
        gc_exec_linef(true, "G53 G0 X%0.3f Y%0.3f Z%0.3f", tool[new_tool].mpos[X_AXIS], tool[new_tool].mpos[Y_AXIS], top_of_z);
    }
    current_tool = 0;

    if (new_tool == 0) {  // if changing to tool 0...we are done.
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Changed to Tool 0");

        // Move Z to Top
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t Move Z to Top");
        gc_exec_linef(true, "G53 G0 Z%0.3f", top_of_z);  
        // Move to Toolsetter
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t Move over Toolsetter");
        gc_exec_linef(true, "G53 G0 X%0.3f Y%0.3f Z%0.3f", tool[new_tool].mpos[X_AXIS], tool[new_tool].mpos[Y_AXIS], top_of_z);
        current_tool = new_tool;
        return true;
    }

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Loading Tool:%d", new_tool);

    // Move Z to Top
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t Move Z to Top");
    gc_exec_linef(true, "G53 G0 Z%0.3f", top_of_z);  

    // Go over new tool
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t Go over new tool");
    gc_exec_linef(true, "G53 G0 X%0.3f Y%0.3f Z%0.3f", tool[new_tool].mpos[X_AXIS]+RELEASE_OFFSET_X, tool[new_tool].mpos[Y_AXIS]+RELEASE_OFFSET_Y, tool[new_tool].mpos[Z_AXIS]+RELEASE_OFFSET_Z);

    set_ATC_open(true);                                               // open ATC
    // drop down to tool
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t drop down to tool");
    gc_exec_linef(true, "G53 G1 F300 X%0.3f Y%0.3f Z%0.3f", tool[new_tool].mpos[X_AXIS], tool[new_tool].mpos[Y_AXIS], tool[new_tool].mpos[Z_AXIS]);
    set_ATC_open(false);                                              // Close ATC
    gc_exec_linef(true, "G4P%0.2f", TOOL_GRAB_TIME);                  // wait for grab to complete and settle

    // move out of toolclip with grabed tool
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t move out of toolclip with grabed tool");
    gc_exec_linef(true, "G53 G1 F300 X%0.3f Y%0.3f Z%0.3f", tool[new_tool].mpos[X_AXIS]+LOAD_OFFSET_X, tool[new_tool].mpos[Y_AXIS]+LOAD_OFFSET_Y, tool[new_tool].mpos[Z_AXIS]+LOAD_OFFSET_Z);
        
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t Move Z Up");
    gc_exec_linef(true, "G53 G0 Z%0.3f", tool[new_tool].mpos[Z_AXIS]+RELEASE_OFFSET_Z);  

    current_tool = new_tool;
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Tool Loaded:%d", new_tool);

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Probe new Tool");
    if (!atc_ETS()) {  // check the length of the tool
        return false;
    }

    // If the spindle was on before we started, we need to turn it back on.
    if (spindle_was_on) {
        gc_exec_linef(false, "M3");
        spindle_spin_delay = esp_timer_get_time() + (spindle_delay_spinup->get() * 1000.0);  // When will spindle spindown be done
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Spindle On");
    }

    // return to saved mpos in XY
    gc_exec_linef(false, "G53G0X%0.3fY%0.3fZ%0.3f", saved_mpos[X_AXIS], saved_mpos[Y_AXIS], top_of_z);

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Back to Work");

    /*

    // return to saved mpos in Z if it is not outside of work area.
    float adjusted_z = saved_mpos[Z_AXIS] + gc_state.tool_length_offset;
    if (adjusted_z < limitsMaxPosition(Z_AXIS)) {
        gc_exec_linef(
            false, "G53G0X%0.3fY%0.3fZ%0.3f", saved_mpos[X_AXIS], saved_mpos[Y_AXIS], saved_mpos[Z_AXIS] + gc_state.tool_length_offset);
    }

    */

    // was was_incremental on? If so, return to that state
    if (was_incremental_mode) {
        gc_exec_linef(false, "G91");
    }


    // Wait for spinup
    if (spindle_was_on) {
        uint64_t current_time = esp_timer_get_time();
        if (current_time < spindle_spin_delay) {
            vTaskDelay(spindle_spin_delay - current_time);
        }
    }
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Toolchange Done");

    return true;
}

void user_tool_change(uint8_t new_tool) {
    if (!atc_tool_change(new_tool, true)) {  // (weak)   should be user defined
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Error, "Tool change failed");
    }
}

void user_probe_notification() {
    float probe_position[MAX_N_AXIS];

    if (sys.state == State::Alarm) {
        return;  // probe failed
    }

    if (tool_setter_probing) {
        return;  // ignore these probes. They are handled elsewhere.
    }

    zeroed_tool_index = current_tool;
}



void user_defined_macro(uint8_t index) {
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Macro: %d", index);
    switch (index) {
        case 0:
            atc_manual_change();
            break;
        default:
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Undefined macro number:%d", index);
            break;
    }
}


// ============= Local functions ==================$H

bool return_tool(uint8_t tool_num) {
    if (tool_num == 0) {
        return false;
    }
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Returning Tool:%d", tool_num);

    // Go to top of Z travel
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t Go to top of Z travel");
    gc_exec_linef(false, "G53G0Z%0.3f", top_of_z);  

    if (current_tool != 0) {
        // move in front of toolclip
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t move in front of toolclip");
        gc_exec_linef(false, "G53 G0 X%0.3f Y%0.3f", tool[tool_num].mpos[X_AXIS]+LOAD_OFFSET_X, tool[tool_num].mpos[Y_AXIS]+LOAD_OFFSET_Y);
        gc_exec_linef(false, "G53G0Z%0.3f", tool[tool_num].mpos[Z_AXIS]+LOAD_OFFSET_Z);  
    }

    // Move into toolclip
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t Move into toolclip");
    gc_exec_linef(true, "G53 G1 F300 X%0.3f Y%0.3f", tool[tool_num].mpos[X_AXIS], tool[tool_num].mpos[Y_AXIS]);  

    set_ATC_open(true);

    // Move on top of Toolclip
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t Move on top of Toolclip");
    gc_exec_linef(true, "G53 G1 F300 X%0.3f Y%0.3f Z%0.3f", tool[tool_num].mpos[X_AXIS]+RELEASE_OFFSET_X, tool[tool_num].mpos[Y_AXIS]+RELEASE_OFFSET_Y, tool[tool_num].mpos[Z_AXIS]+RELEASE_OFFSET_Z);
    set_ATC_open(false);   // close ATC

    return true;
}

bool atc_ETS() {
    float probe_to;  // Calculated work position
    float probe_position[MAX_N_AXIS];

    // move over ETS
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "\t\t move over ETS");
    gc_exec_linef(false, "G53 G0 X%0.3f Y%0.3f Z%0.3f", tool[ETS_INDEX].mpos[X_AXIS], tool[ETS_INDEX].mpos[Y_AXIS], tool[ETS_INDEX].mpos[Z_AXIS]+RELEASE_OFFSET_Z);

    atc_ETS_dustoff();

    //grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Probing disabled for testing");


    float wco = gc_state.coord_system[Z_AXIS] + gc_state.coord_offset[Z_AXIS] + gc_state.tool_length_offset;
    probe_to  = tool[ETS_INDEX].mpos[Z_AXIS] - wco;

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Probing to %0.3f", probe_to);


    // https://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G38-probe
    tool_setter_probing = true;
    gc_exec_linef(true, "G38.2F%0.3fZ%0.3f", 150.0, probe_to);  // probe
    tool_setter_probing = false;

    // Was probe successful?
    if (sys.state == State::Alarm) {
        if (sys_rt_exec_alarm == ExecAlarm::ProbeFailInitial) {
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Probe Switch Error");
        } else {
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Missing Tool?");
        }
        return false;  // fail
    }

    system_convert_array_steps_to_mpos(probe_position, sys_probe_position);
    tool[current_tool].offset[Z_AXIS] = probe_position[Z_AXIS];  // Get the Z height ...

    if (zeroed_tool_index != 0) {
        float tlo = tool[current_tool].offset[Z_AXIS] - tool[zeroed_tool_index].offset[Z_AXIS];
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Tool No:%d TLO:%0.3f", current_tool, tlo);
        // https://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G43_1
        gc_exec_linef(false, "G43.1Z%0.3f", tlo);  // raise up
    }


    gc_exec_linef(false, "G53G0Z%0.3f", top_of_z);  // raise up

    return true;
}

bool set_ATC_open(bool open) {
    // todo lots of safety checks
    if (gc_state.modal.spindle != SpindleState::Disable) {
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Fail spindle on during change");
        return false;
    }
    digitalWrite(ATC_RELEASE_PIN, open);
    if (open)
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Release");
    else
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Close");
    return true;
}

// give a squirt of air to clear top of Tool Setter
bool atc_ETS_dustoff() {
#ifdef ETS_DUST_OFF
    digitalWrite(ETS_DUST_OFF, HIGH);
    gc_exec_linef(true, "G4P%0.2f", ETS_DUST_OFF_DURATION);
    digitalWrite(ETS_DUST_OFF, LOW);
#endif
    return true;
}

bool atc_manual_change() {
    // if (gc_state.modal.spindle != SpindleState::Disable) {
    //     grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Cannot use ATC with spindle on");
    // }

    if (sys.state != State::Idle) {
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC manual change only permitted in idle");
        return false;
    }

    return set_ATC_open(!digitalRead(ATC_RELEASE_PIN));   // Toggle ATC Pin
    /*
    if (!set_ATC_open(true)) {  // internally checks spindle state
        return false;
    }

    vTaskDelay(ATC_MANUAL_CHANGE_TIME);

    if (!set_ATC_open(false)) {
        return false;
    }

    return true;

    */
}

/*
    Format and send gcode line with optional synchronization
    sync_after: Forces all buffered lines to be completed for line send
    format: a printf style string

*/
//void grbl_msg_sendf(uint8_t client, MsgLevel level, const char* format, ...);
void gc_exec_linef(bool sync_after, const char* format, ...) {
    char    loc_buf[100];
    char*   temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    size_t len = vsnprintf(NULL, 0, format, arg);
    va_end(copy);

    if (len >= sizeof(loc_buf)) {
        temp = new char[len + 1];
        if (temp == NULL) {
            return;
        }
    }
    len = vsnprintf(temp, len + 1, format, arg);

    gc_execute_line(temp, CLIENT_INPUT);
    //grbl_sendf(CLIENT_SERIAL, "[ATC GCode:%s]\r\n", temp);
    va_end(arg);
    if (temp != loc_buf) {
        delete[] temp;
    }
    if (sync_after) {
        protocol_buffer_synchronize();
    }
}
