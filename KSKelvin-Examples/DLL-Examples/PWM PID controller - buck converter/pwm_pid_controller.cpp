// Automatically generated C++ file on Thu Oct 29 16:42:43 2026
//
// To build with Digital Mars C++ Compiler:
//
//    dmc -mn -WD -o pwm_pid_controller.cpp kernel32.lib

#include <malloc.h>

extern "C" __declspec(dllexport) void (*Display)(const char *format, ...)       = 0; // works like printf()
extern "C" __declspec(dllexport) void (*EXIT)(const char *format, ...)          = 0; // print message like printf() but exit(0) afterward
extern "C" __declspec(dllexport) const double *DegreesC                         = 0; // pointer to current circuit temperature
extern "C" __declspec(dllexport) const int *StepNumber                          = 0; // pointer to current step number
extern "C" __declspec(dllexport) const int *NumberSteps                         = 0; // pointer to estimated number of steps
extern "C" __declspec(dllexport) const char* const *InstanceName                = 0; // pointer to address of instance name
extern "C" __declspec(dllexport) const char *QUX                                = 0; // path to QUX.exe
extern "C" __declspec(dllexport) const bool *ForKeeps                           = 0; // pointer to whether being evaluated non-hypothetically
extern "C" __declspec(dllexport) const bool *HoldICs                            = 0; // pointer to whether instance initial conditions are being held
extern "C" __declspec(dllexport) const void *GUI_HWND                           = 0; // pointer to Window handle of QUX.exe
extern "C" __declspec(dllexport) const double *CKTtime                          = 0;
extern "C" __declspec(dllexport) const double *CKTdelta                         = 0;
extern "C" __declspec(dllexport) const int *IntegrationOrder                    = 0;
extern "C" __declspec(dllexport) const char *InstallDirectory                   = 0;
extern "C" __declspec(dllexport) double (*EngAtof)(const char **string)         = 0;
extern "C" __declspec(dllexport) const char *(*BinaryFormat)(unsigned int data)                          = 0; // BinaryFormat(0x1C) returns "0b00011100"
extern "C" __declspec(dllexport) const char *(*EngFormat   )(double x, const char *units, int numDgts)   = 0; // EngFormat(1e-6, "s", 6) returns "1µs"
extern "C" __declspec(dllexport) int (*DFFT)(struct sComplex *u, bool inv, unsigned int N, double scale) = 0; // Discrete Fast Fourier Transform
extern "C" __declspec(dllexport) void (*bzero)(void *ptr, unsigned int count)   = 0;

union uData
{
   bool b;
   char c;
   unsigned char uc;
   short s;
   unsigned short us;
   int i;
   unsigned int ui;
   float f;
   double d;
   long long int i64;
   unsigned long long int ui64;
   char *str;
   unsigned char *bytes;
};

// int DllMain() must exist and return 1 for a process to load the .DLL
// See https://docs.microsoft.com/en-us/windows/win32/dlls/dllmain for more information.
int __stdcall DllMain(void *module, unsigned int reason, void *reserved) { return 1; }

// #undef pin names lest they collide with names in any header file(s) you might include.
#undef setpt
#undef ctrl
#undef fb
#undef sawtooth
#undef Q

#include <math.h>
double carrier(int type,double t, double fsw, double phase){
   // Generate periodic carrier waveforms for PWM modulation
   // Inputs:
   //   type  : Waveform type selector
   //   t     : Simulation time [seconds]
   //   fsw   : Switching frequency [Hz] (carrier frequency)
   //   phase : Phase shift [radians] (offset applied to carrier waveform)

   // type 0: Sinusoidal Carrier
   if      (type == 0) return sin(2*M_PI*fsw*(t-phase/2/M_PI/fsw));
   // type 1: Sawtooth wave (0 to 1V) - increases linearly then resets
   else if (type == 1) return fmod(t-phase/2/M_PI/fsw,1/fsw)*fsw;
   // type 2: Inverted Sawtooth wave (1 to 0V) - decreases linearly then resets
   else if (type == 2) return 1-fmod(t-+phase/2/M_PI/fsw,1/fsw)*fsw;
   // type 3: Triangle wave (0 to 1V) - symmetrical about 0.5V
   else if (type == 3) return fabs(remainder(t-phase/2/M_PI/fsw,1/fsw))*2*fsw;
   // type 4: Triangle wave (-1 to +1V) - bipolar symmetrical
   else if (type == 4) return fabs(remainder(t-phase/2/M_PI/fsw,1/fsw))*4*fsw-1;
   // Unknown type: return 0 (safe default to prevent unintended behavior)
   else                return 0;
}

struct sPWM_PID_CONTROLLER
{
  // declare the structure here
   bool   init;        // one-time init flag (set on first call)
   double maxstep;     // max solver step for PID accuracy
   double nextTsw;     // scheduled time of next switching-cycle event
   double Fpid;        // PID controller sampling frequency
   double nextTpid;    // scheduled time of next PID tick
   double error_n1;    // error[n-1]   (previous error sample)
   double errorI_n1;   // errorI[n-1]  (previous integral accumulator)
   bool   prevQ;       // previous Q output
   bool   latch;       // latch flag: true = Q frozen for rest of cycle
   double prevSAW;     // previous carrier value
};

extern "C" __declspec(dllexport) void pwm_pid_controller(struct sPWM_PID_CONTROLLER **opaque, double t, union uData *data)
{
   double  setpt    = data[ 0].d; // input
   double  fb       = data[ 1].d; // input
   double  Kp       = data[ 2].d; // input parameter
   double  Ki       = data[ 3].d; // input parameter
   double  Kd       = data[ 4].d; // input parameter
   double  Kv       = data[ 5].d; // input parameter
   bool    Itype    = data[ 6].b; // input parameter
   double  Fsw      = data[ 7].d; // input parameter
   double  OSD      = data[ 8].d; // input parameter
   double &ctrl     = data[ 9].d; // output
   double &sawtooth = data[10].d; // output
   bool   &Q        = data[11].b; // output

   if(!*opaque)
   {
      *opaque = (struct sPWM_PID_CONTROLLER *) malloc(sizeof(struct sPWM_PID_CONTROLLER));
      bzero(*opaque, sizeof(struct sPWM_PID_CONTROLLER));
   }
   struct sPWM_PID_CONTROLLER *inst = *opaque;

// Implement module evaluation code here:
   const double Fpid = Fsw * OSD;   // PID sampling freq = switching freq * oversampling ratio

   if (!inst->init) {
      inst->maxstep = 1 / Fpid / 100;   // suggested max solver step (200 pts per PID period)
      inst->init = true;
   }

   sawtooth = carrier(1, t, Fsw, 0);   // normalized triangle/saw carrier, 0..1

   // Detect carrier valley (falling through 0.5). Uses prevSAW vs current SAW.
   const bool valley = (inst->prevSAW > 0.5) && (sawtooth <= 0.5);

   if (valley) {
       inst->latch = false;      // re-arm the latch once per switching cycle
   }

   // Latch comparator result only while unlatched (i.e. from valley until Q goes low).
   if (!inst->latch) {
       Q = (ctrl > sawtooth);    // PWM comparator: high when control > carrier
       if (Q == false) {
          inst->latch = true;    // freeze Q=0 for the rest of the cycle
       }
   }
   inst->prevQ   = Q;            // remember last output
   inst->prevSAW = sawtooth;     // remember last carrier value (for valley detect)

   // ---------------- PID update (runs once per PID period) ----------------
   if (t >= inst->nextTpid)      // scheduled PID tick (rising edge of control clock)
   {
      //inst->prevQ = true;
      double Tpid = 1 / Fpid;   // PID period
      inst->nextTpid = inst->nextTpid + 1 / Fpid;   // schedule next PID tick

      // calculate error (scaled setpoint - feedback)
      double error = Kv * (setpt - fb);

      // proportional term
      double errorP = error;

      // integral term: trapezoidal or rectangular integration
      double errorI;
      if (Itype)                // trapezoidal rule (more accurate)
      errorI = (error + inst->error_n1) * Tpid / 2 + inst->errorI_n1;
      else                      // rectangular (backward Euler) method
      errorI = error * Tpid + inst->errorI_n1;

      // derivative term (backward difference; matches MATLAB 'matched' form)
      double errorD = (error - inst->error_n1) / Tpid;

      // PID formula
      ctrl = Kp * errorP + Ki * errorI + Kd * errorD;

      // store [n-1] sampling for next iteration
      inst->error_n1  = error;    // error[n-1] = error[n]
      inst->errorI_n1 = errorI;   // errorI[n-1] = errorI[n]
   }
}

extern "C" __declspec(dllexport) double MaxExtStepSize(struct sPWM_PID_CONTROLLER *inst, double t)
{
   return inst->maxstep;
}

extern "C" __declspec(dllexport) void Trunc(struct sPWM_PID_CONTROLLER *inst, double t, union uData *data, double *timestep)
{ // limit the timestep to a tolerance if the circuit causes a change in struct sPWM_PID_CONTROLLER
   const double ttol = 1e-9; // 1ns default tolerance
   if(*timestep > ttol)
   {
      struct sPWM_PID_CONTROLLER tmp = *inst;
      pwm_pid_controller(&(&tmp), t, data);

      if(tmp.prevQ != inst->prevQ)  // Q toggle reduce timestep to TTOL
         *timestep = ttol;
   }
}

extern "C" __declspec(dllexport) void Destroy(struct sPWM_PID_CONTROLLER *inst)
{
   free(inst);
}
