
#include "SyncWithPreamble.h"

SyncWithPreamble::SyncWithPreamble(Zippy* z)
  : zippy(z),
    lighthouse(z->getLighthouse())
{

}

void SyncWithPreamble::start(unsigned long currentTime)
{
  zippy->stop();
  lighthouse->clearPreambleFlag();
}

bool SyncWithPreamble::loop(unsigned long currentTime)
{
  return lighthouse->foundPreamble();
}
