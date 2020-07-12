// Log, version 0.1: a simple logging class
#include <ctime>
#include <iostream>

enum TLogLevel {logERROR, logWARNING, logINFO, logDEBUG, logDEBUG1,
logDEBUG2, logDEBUG3, logDEBUG4};
class Log
{
public:
   Log(){};
   virtual ~Log();
   std::ostringstream& Get(TLogLevel level = logINFO);
public:
   static TLogLevel& ReportingLevel(){};
protected:
   std::ostringstream os;
private:
   Log(const Log&);
   Log& operator =(const Log&);
private:
   TLogLevel messageLevel;
};
std::ostringstream& Log::Get(TLogLevel level)
{
   os << "- " << std::time(0);
   //os << " " << ToString(level) << ": ";
   os << std::string(level > logDEBUG ? 0 : level - logDEBUG, '\t');
   messageLevel = level;
   return os;
}
Log::~Log()
{
   if (messageLevel >= Log::ReportingLevel())
   {
      os << std::endl;
      fprintf(stderr, "%s", os.str().c_str());
      fflush(stderr);
   }
}
