#ifndef __TMP__
#define __TMP__

#include "CommonDef.h"
#include "Picture.h"

#if JVET_AK0085_TM_BOUNDARY_PADDING
struct IdxCost
{
  int i;
  int k;
  int32_t cost;
};
typedef int16_t BoundaryTop;
typedef float   BoundaryBottom;
typedef int32_t BoundaryLeft;
typedef long    BoundaryRight;

void addCand(std::vector<IdxCost>& bestCands, int i, int k, int32_t cost, const int numAvgCands);

template<typename T> void TemplateMatchingPadding(PelUnitBuf unitBuf, T boundary, Area subpicArea)
{
  ChromaFormat cf = unitBuf.chromaFormat;
  // candidate definition
  const int NUM_STEPS = 16;
  const int numCands[NUM_STEPS] = {1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31};
  const int NUM_CANDS = 31;
  const int candPos[NUM_STEPS][2][NUM_CANDS] = {{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},
                                                {{1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},
                                                {{2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 2, 2, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},
                                                {{3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 2, 2, 3, 3, 4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},
                                                {{4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 2, 2, 3, 3, 4, 4, 5, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},
                                                {{5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},
                                                {{6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},
                                                {{7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},
                                                {{8, -8, 7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},
                                                {{9, -9, 8, -8, 7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},
                                                {{10, -10, 9, -9, 8, -8, 7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},
                                                {{11, -11, 10, -10, 9, -9, 8, -8, 7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 1, 1, 1, 1, 1, 1, 1, 1}},
                                                {{12, -12, 11, -11, 10, -10, 9, -9, 8, -8, 7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 1, 1, 1, 1, 1, 1}},
                                                {{13, -13, 12, -12, 11, -11, 10, -10, 9, -9, 8, -8, 7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0}, {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 1, 1, 1, 1}},
                                                {{14, -14, 13, -13, 12, -12, 11, -11, 10, -10, 9, -9, 8, -8, 7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0}, {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 1, 1}},
                                                {{15, -15, 14, -14, 13, -13, 12, -12, 11, -11, 10, -10, 9, -9, 8, -8, 7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0}, {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16}}};

  static constexpr bool isBotTop = std::is_same_v<T, BoundaryBottom> || std::is_same_v<T, BoundaryTop>;

  const int indWidth = isBotTop ? subpicArea.size().width : subpicArea.size().height;
  const int indStride = TMP_TEMPLATE_WIDTH - TMP_TW_MINUS_TBW_BY_TWO - TMP_TW_MINUS_TBW_BY_TWO;

  Size lumaKernelSize;
  if constexpr(isBotTop)
  {
    lumaKernelSize = Size(TMP_TEMPLATE_WIDTH, 1);
  }
  else
  {
    lumaKernelSize = Size(1, TMP_TEMPLATE_WIDTH);
  }

  CompStorage pixStorage = CompStorage();
  Area area = Area(Position(0, 0), lumaKernelSize);
  pixStorage.create(area);
  PelBuf pix = PelBuf(pixStorage);

  int currLine = 0;

  const int idxStop = (indWidth - TMP_TW_MINUS_TBW_BY_TWO + indStride - 1) / indStride;

  int*** candidateArray = new int**[idxStop];
  for (int i = 0; i < idxStop; ++i)
  {
    candidateArray[i] = new int*[TMP_NUM_AVG_CANDS];
    for (int j = 0; j < TMP_NUM_AVG_CANDS; ++j)
    {
      candidateArray[i][j] = new int[2];
    }
  }

  int ind = 0;
  for(int idx=0; idx<idxStop; idx++)
  {
    if(ind > indWidth - TMP_TEMPLATE_WIDTH)
    {
      ind = indWidth - TMP_TEMPLATE_WIDTH;
    }

    int costs[TMP_MAX_NUM_STEPS][NUM_CANDS] = { };
    for(int s=0; s<TMP_MAX_NUM_STEPS; s++)
    {
      for(int c=0; c<NUM_CANDS; c++)
      {
        costs[s][c] = -1;
      }
    }

    int minCost = MAX_INT;
    int comp = 0;

    PelBuf buf = unitBuf.get(ComponentID(comp));

    PelBuf kernel;
    if constexpr(std::is_same_v<T, BoundaryTop>)
    {
      kernel = buf.subBuf(Position(ind+subpicArea.pos().x, subpicArea.pos().y - currLine), Size(TMP_TEMPLATE_WIDTH, TMP_TEMPLATE_HEIGHT));
    }
    else if constexpr(std::is_same_v<T, BoundaryBottom>)
    {
      kernel = buf.subBuf(Position(ind+subpicArea.pos().x, subpicArea.pos().y + subpicArea.size().height + currLine - TMP_TEMPLATE_HEIGHT), Size(TMP_TEMPLATE_WIDTH, TMP_TEMPLATE_HEIGHT));
    }
    else if constexpr(std::is_same_v<T, BoundaryLeft>)
    {
      kernel = buf.subBuf(Position(subpicArea.pos().x - currLine, ind+subpicArea.pos().y), Size(TMP_TEMPLATE_HEIGHT, TMP_TEMPLATE_WIDTH));
    }
    else if constexpr(std::is_same_v<T, BoundaryRight>)
    {
      kernel = buf.subBuf(Position(subpicArea.pos().x + subpicArea.size().width + currLine - TMP_TEMPLATE_HEIGHT, ind+subpicArea.pos().y), Size(TMP_TEMPLATE_HEIGHT, TMP_TEMPLATE_WIDTH));
    }

    for ( int k = 0; k <TMP_MAX_NUM_STEPS; k++)
    {
      for ( int i = 0; i <numCands[k]; i++)
      {
        int tempCost = 0;
        if(ind+candPos[k][0][i] < 0 || ind+candPos[k][0][i]+TMP_TEMPLATE_WIDTH > indWidth)
        {
          continue;
        }

        // get cand
        PelBuf cand;
        if constexpr(std::is_same_v<T, BoundaryTop>)
        {
          cand = buf.subBuf(Position(ind + subpicArea.pos().x + candPos[k][0][i], subpicArea.pos().y - currLine + candPos[k][1][i]), Size(TMP_TEMPLATE_WIDTH, TMP_TEMPLATE_HEIGHT));
        }
        else if constexpr(std::is_same_v<T, BoundaryBottom>)
        {
          cand = buf.subBuf(Position(ind + subpicArea.pos().x + candPos[k][0][i], subpicArea.pos().y + subpicArea.size().height + currLine - TMP_TEMPLATE_HEIGHT - candPos[k][1][i]), Size(TMP_TEMPLATE_WIDTH, TMP_TEMPLATE_HEIGHT));
        }
        else if constexpr(std::is_same_v<T, BoundaryLeft>)
        {
          cand = buf.subBuf(Position(subpicArea.pos().x - currLine + candPos[k][1][i], ind + subpicArea.pos().y + candPos[k][0][i]), Size(TMP_TEMPLATE_HEIGHT, TMP_TEMPLATE_WIDTH));
        }
        else if constexpr(std::is_same_v<T, BoundaryRight>)
        {
          cand = buf.subBuf(Position(subpicArea.pos().x + subpicArea.size().width + currLine - TMP_TEMPLATE_HEIGHT - candPos[k][1][i], ind + subpicArea.pos().y + candPos[k][0][i]), Size(TMP_TEMPLATE_HEIGHT, TMP_TEMPLATE_WIDTH));
        }

        for(int n = 0; n<TMP_TEMPLATE_HEIGHT; n++)
        {
          for(int m = 0; m<TMP_TEMPLATE_WIDTH; m++)
          {
            Pel* a;
            Pel* b;

            if constexpr(isBotTop)
            {
              a = cand.bufAt(m, n);
              b = kernel.bufAt(m, n);
            }
            else
            {
              a = cand.bufAt(n, m);
              b = kernel.bufAt(n, m);
            }

            tempCost += std::abs(int32_t(*a) - int32_t(*b));
          }
        }
        costs[k][i] = tempCost;
        if(tempCost<minCost)
        {
          minCost = tempCost;
        }
      }
      if(k >= TMP_MIN_NUM_STEPS)
      {
        if(minCost <= TMP_EARLY_TERMINATION_THRESHOLD)
        {
          break;
        }
      }
    }
    // adaptive candidate selection
    std::vector<IdxCost> bestCands;
    int scaledThres = int(TMP_ACS_FACTOR*float(minCost));
      

    for ( int k = 0; k <TMP_MAX_NUM_STEPS; k++)
    {
      for(int i=0; i<NUM_CANDS; i++)
      {
        int cost = costs[k][i];
        if(cost>=0 && cost <= scaledThres)
        {
          addCand(bestCands, i, k, cost, TMP_NUM_AVG_CANDS);
        }
      }
    }

    for(int n = 0; n<TMP_NUM_AVG_CANDS; n++)
    {
      if(n<bestCands.size())
      {
        candidateArray[idx][n][0] = bestCands.at(n).i;
        candidateArray[idx][n][1] = bestCands.at(n).k;
      }
      else
      {
        candidateArray[idx][n][0] = -1;
        candidateArray[idx][n][1] = -1;
      }
    }
    ind += indStride;
  }

  for(int pl=0; pl<TMP_PADSIZE; pl++)
  {
    currLine = pl;
    ind = 0;
    for(int idx=0; idx<idxStop; idx++)
    {
      if(ind > indWidth - TMP_TEMPLATE_WIDTH)
      {
        ind = indWidth - TMP_TEMPLATE_WIDTH;
      }

      const int comp = 0;
      PelBuf buf = unitBuf.get(ComponentID(comp));

      pix.fill(0);

      int mStart = ind == 0 ? 0 : TMP_TW_MINUS_TBW_BY_TWO;
      int mEnd = ind == indWidth - TMP_TEMPLATE_WIDTH ? TMP_TEMPLATE_WIDTH : TMP_TEMPLATE_WIDTH - TMP_TW_MINUS_TBW_BY_TWO;

      int numSelCands = 0;
      for(int n = 0; n<TMP_NUM_AVG_CANDS; n++)
      {
        int i = candidateArray[idx][n][0];
        int k = candidateArray[idx][n][1];
        if(i<0)
        {
          break;
        }
        numSelCands++;


        Area sourceArea;
        if constexpr(std::is_same_v<T, BoundaryTop>)
        {
          Position lumaPos = Position(ind + subpicArea.pos().x + candPos[k][0][i], subpicArea.pos().y - currLine + candPos[k][1][i] - 1);
          sourceArea = Area(lumaPos, lumaKernelSize);
        }
        else if constexpr(std::is_same_v<T, BoundaryBottom>)
        {
          Position lumaPos = Position(ind + subpicArea.pos().x + candPos[k][0][i], subpicArea.pos().y + subpicArea.size().height + currLine - candPos[k][1][i]);
          sourceArea = Area(lumaPos, lumaKernelSize);
        }
        else if constexpr(std::is_same_v<T, BoundaryLeft>)
        {
          Position lumaPos = Position(subpicArea.pos().x - currLine + candPos[k][1][i] - 1, ind + subpicArea.pos().y + candPos[k][0][i]);
          sourceArea = Area(lumaPos, lumaKernelSize);
        }
        else if constexpr(std::is_same_v<T, BoundaryRight>)
        {
          Position lumaPos = Position(subpicArea.pos().x + subpicArea.size().width + currLine - candPos[k][1][i], ind + subpicArea.pos().y + candPos[k][0][i]);
          sourceArea = Area(lumaPos, lumaKernelSize);
        }

        PelBuf source = buf.subBuf(sourceArea.pos(), sourceArea.size());
        for(int m = mStart; m<mEnd; m++)
        {
          if (isBotTop)
          {
          *pix.bufAt(m, 0) += *source.bufAt(m, 0);
          }
          else
          {
          *pix.bufAt(0, m) += *source.bufAt(0, m);
          }
        }
      }
      for(int m = mStart; m<mEnd; m++)
      {
        if (isBotTop)
        {
          *pix.bufAt(m, 0) /= numSelCands;
        }
        else
        {
          *pix.bufAt(0, m) /= numSelCands;
        }
      }
        

      Position lumaDstPos;
      if constexpr(std::is_same_v<T, BoundaryTop>)
      {
        lumaDstPos = Position(ind + subpicArea.pos().x, subpicArea.pos().y - currLine - 1);
      }
      else if constexpr(std::is_same_v<T, BoundaryBottom>)
      {
        lumaDstPos = Position(ind + subpicArea.pos().x, subpicArea.pos().y + subpicArea.size().height + currLine);
      }
      else if constexpr(std::is_same_v<T, BoundaryLeft>)
      {
        lumaDstPos = Position(subpicArea.pos().x - currLine - 1, ind + subpicArea.pos().y);
      }
      else if constexpr(std::is_same_v<T, BoundaryRight>)
      {
        lumaDstPos = Position(subpicArea.pos().x + subpicArea.size().width + currLine, ind + subpicArea.pos().y);
      }

      PelBuf subBuf = buf.subBuf(lumaDstPos, lumaKernelSize);

      for(int m = mStart; m<mEnd; m++)
      {
        if (isBotTop)
        {
          *subBuf.bufAt(m, 0) = *pix.bufAt(m, 0);
        }
        else
        {
          *subBuf.bufAt(0, m) = *pix.bufAt(0, m);
        }
      }
      ind += indStride;
    }
  }



  bool processChroma = true;
  int ssX = 0;
  int ssY = 0;
  if(cf == CHROMA_400)
  {
    processChroma = false;
  }
  if(cf == CHROMA_420)
  {
    ssX = 1;
    ssY = 1;
  }
  if(cf == CHROMA_422)
  {
    ssX = 1;
    ssY = 0;
  }
  if(cf == CHROMA_444)
  {
    ssX = 0;
    ssY = 0;
  }

  int plStride;
  int ssBoundary;
  if constexpr(isBotTop)
  {
    plStride =  ssY ? 2 : 1;
    ssBoundary =  ssX;
  }
  else
  {
    plStride =  ssX ? 2 : 1;
    ssBoundary =  ssY;
  }

  Size chromaKernelSize;
  if constexpr(isBotTop)
  {
    chromaKernelSize = Size(TMP_TEMPLATE_WIDTH>>ssX, 1);
  }
  else
  {
    chromaKernelSize = Size(1, TMP_TEMPLATE_WIDTH>>ssY);
  }

  CompStorage pixStorageCb = CompStorage();
  CompStorage pixStorageCr = CompStorage();
  pixStorageCb.create(Area(Position(0, 0), chromaKernelSize));
  pixStorageCr.create(Area(Position(0, 0), chromaKernelSize));
  PelBuf pixCb = PelBuf(pixStorageCb);
  PelBuf pixCr = PelBuf(pixStorageCr);

  if(processChroma)
  {
    for(int pl=0; pl<TMP_PADSIZE; pl+=plStride)
    {
      currLine = pl;
      ind = 0;
      for(int idx=0; idx<idxStop; idx++)
      {
        if(ind > indWidth - TMP_TEMPLATE_WIDTH)
        {
          ind = indWidth - TMP_TEMPLATE_WIDTH;
        }


        pixCb.fill(0);
        pixCr.fill(0);

        const int mStart = ind == 0 ? 0 : TMP_TW_MINUS_TBW_BY_TWO >> ssBoundary;
        const int mEnd = ind == indWidth - TMP_TEMPLATE_WIDTH ? TMP_TEMPLATE_WIDTH >> ssBoundary : (TMP_TEMPLATE_WIDTH - TMP_TW_MINUS_TBW_BY_TWO) >> ssBoundary;

        int candCounter = 0;
        for(int indAdd=0; indAdd<(1+ssBoundary); indAdd++)
        {
          for(int n = 0; n<TMP_NUM_AVG_CANDS; n++)
          {

            int i = candidateArray[idx][n][0];
            int k = candidateArray[idx][n][1];

            int ix = candPos[k][0][i];
            int iy = candPos[k][1][i];

            if(i < 0)
            {
              break;
            }
            else if(iy <= 1)
            {
              ix += candPos[k][0][i];
              iy += 1;
            }
            if(ind+indAdd+ix < 0 || ind+indAdd+ix+TMP_TEMPLATE_WIDTH > indWidth)
            {
              ix = candPos[k][0][i];
            }
            candCounter++;

            Area sourceArea;
            Area sourceAreaChroma;
            if constexpr(std::is_same_v<T, BoundaryTop>)
            {
              Position lumaPos = Position(ind + indAdd + subpicArea.pos().x + ix, subpicArea.pos().y - currLine + iy - 1);
              Position chromaPos = Position(lumaPos.x >> ssX, lumaPos.y >> ssY);
              sourceAreaChroma = Area(chromaPos, chromaKernelSize);
            }
            else if constexpr(std::is_same_v<T, BoundaryBottom>)
            {
              Position lumaPos = Position(ind + indAdd + subpicArea.pos().x + ix, subpicArea.pos().y + subpicArea.size().height + currLine - iy);
              Position chromaPos = Position(lumaPos.x >> ssX, lumaPos.y >> ssY);
              sourceAreaChroma = Area(chromaPos, chromaKernelSize);
            }
            else if constexpr(std::is_same_v<T, BoundaryLeft>)
            {
              Position lumaPos = Position(subpicArea.pos().x - currLine + iy - 1, ind +indAdd + subpicArea.pos().y + ix);
              Position chromaPos = Position(lumaPos.x >> ssX, lumaPos.y >> ssY);
              sourceAreaChroma = Area(chromaPos, chromaKernelSize);
            }
            else if constexpr(std::is_same_v<T, BoundaryRight>)
            {
              Position lumaPos = Position(subpicArea.pos().x + subpicArea.size().width + currLine - iy, ind + indAdd + subpicArea.pos().y + ix);
              Position chromaPos = Position(lumaPos.x >> ssX, lumaPos.y >> ssY);
              sourceAreaChroma = Area(chromaPos, chromaKernelSize);
            }

            PelBuf sourceCb = unitBuf.get(COMPONENT_Cb).subBuf(sourceAreaChroma.pos(), sourceAreaChroma.size());
            PelBuf sourceCr = unitBuf.get(COMPONENT_Cr).subBuf(sourceAreaChroma.pos(), sourceAreaChroma.size());
            for(int m = mStart; m<mEnd; m++)
            {
              if (isBotTop)
              {
                *pixCb.bufAt(m, 0) += *sourceCb.bufAt(m, 0);
                *pixCr.bufAt(m, 0) += *sourceCr.bufAt(m, 0);
              }
              else
              {
                *pixCb.bufAt(0, m) += *sourceCb.bufAt(0, m);
                *pixCr.bufAt(0, m) += *sourceCr.bufAt(0, m);
              }
            }
          }
        }
        for(int m = mStart; m<mEnd; m++)
        {
          if(isBotTop)
          {
            *pixCb.bufAt(m, 0) /= candCounter;
            *pixCr.bufAt(m, 0) /= candCounter;
          }
          else
          {
            *pixCb.bufAt(0, m) /= candCounter;
            *pixCr.bufAt(0, m) /= candCounter;
          }
        }

        Position lumaDstPos;
        Position chromaDstPos;
        if constexpr(std::is_same_v<T, BoundaryTop>)
        {
          lumaDstPos = Position(ind + subpicArea.pos().x, subpicArea.pos().y - currLine - 2);
          chromaDstPos = Position(lumaDstPos.x >> ssX, lumaDstPos.y >> ssY);
        }
        else if constexpr(std::is_same_v<T, BoundaryBottom>)
        {
          lumaDstPos = Position(ind + subpicArea.pos().x, subpicArea.pos().y + subpicArea.size().height + currLine);
          chromaDstPos = Position(lumaDstPos.x >> ssX, lumaDstPos.y >> ssY);
        }
        else if constexpr(std::is_same_v<T, BoundaryLeft>)
        {
          lumaDstPos = Position(subpicArea.pos().x - currLine - 2, ind + subpicArea.pos().y);
          chromaDstPos = Position(lumaDstPos.x >> ssX, lumaDstPos.y >> ssY);
        }
        else if constexpr(std::is_same_v<T, BoundaryRight>)
        {
          lumaDstPos = Position(subpicArea.pos().x + subpicArea.size().width + currLine, ind + subpicArea.pos().y);
          chromaDstPos = Position(lumaDstPos.x >> ssX, lumaDstPos.y >> ssY);
        }

        PelBuf dstCb = unitBuf.get(COMPONENT_Cb).subBuf(chromaDstPos, chromaKernelSize);
        PelBuf dstCr = unitBuf.get(COMPONENT_Cr).subBuf(chromaDstPos, chromaKernelSize);
        for(int m = mStart; m<mEnd; m++)
        {
          if (isBotTop)
          {
            *dstCb.bufAt(m, 0) = *pixCb.bufAt(m, 0);
            *dstCr.bufAt(m, 0) = *pixCr.bufAt(m, 0);
          }
          else
          {
            *dstCb.bufAt(0, m) = *pixCb.bufAt(0, m);
            *dstCr.bufAt(0, m) = *pixCr.bufAt(0, m);
          }
        }
        ind += indStride;
      }
    }
  }

  for (int i = 0; i < idxStop; ++i)
  {
    for (int j = 0; j < TMP_NUM_AVG_CANDS; ++j)
    {
      delete[] candidateArray[i][j];
    }
    delete[] candidateArray[i];
  }
  delete[] candidateArray;
};

#endif
#endif