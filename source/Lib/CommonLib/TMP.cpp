#include "TMP.h"


#if JVET_AK0085_TM_BOUNDARY_PADDING
void addCand(std::vector<IdxCost>& bestCands, int i, int k, int32_t cost, const int numAvgCands)
{
  IdxCost currIdx;
  currIdx.i = i;
  currIdx.k = k;
  currIdx.cost = cost;
  if(bestCands.size() == 0)
  {
    bestCands.push_back(currIdx);
  }
  else
  {
    uint64_t idx = bestCands.size();
    for(int n = 0; n < bestCands.size(); ++n)
    {
      if(bestCands[n].cost > cost)
      {
        idx = n;
        break;
      }
    }
    if(idx<bestCands.size())
    {
      bestCands.insert(bestCands.begin()+idx, currIdx);
    }
    else
    {
      bestCands.push_back(currIdx);
    }
  }
  while(bestCands.size()>numAvgCands)
  {
    bestCands.erase(bestCands.begin()+numAvgCands);
  }
}
#endif


