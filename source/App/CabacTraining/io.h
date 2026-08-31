#pragma once
#include <iostream>
#include <vector>
#include "Contexts.h"
#include "data.h"
using namespace std;


extern int verbose;

#if JVET_AH0176_LOW_DELAY_B_CTX
static Models readModelParameters(std::istream &in)
{
  constexpr int nb_model_to_read = kNbModels;
  Models prms;
  assert(nb_model_to_read <= (int)prms.size());
  for (int k = 0; k < nb_model_to_read; ++k)
  {
    in >> prms[k].initId;
  }
  for (int k = 0; k < nb_model_to_read; ++k)
  {
    in >> prms[k].log2windowsize;
  }
  for (int k = 0; k < nb_model_to_read; ++k)
  {
    in >> prms[k].adaptweight;
  }
  in >> prms[0].rateoffset[0] >> prms[0].rateoffset[1];
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
  in >> prms[1].rateoffset[0] >> prms[1].rateoffset[1];
  in >> prms[2].rateoffset[0] >> prms[2].rateoffset[1];
  in >> prms[3].rateoffset[0] >> prms[3].rateoffset[1];
#else
  for (int k = 1; k < 3; ++k)
  {
    prms[k].rateoffset[0] = prms[0].rateoffset[0];
    prms[k].rateoffset[1] = prms[0].rateoffset[1];
  }
#endif
  in.ignore(1024, '\n');
  return prms;
}
#else
static std::array<ModelParameters, 3> readModelParameters( std::istream &in )
{
  std::array<ModelParameters, 3> prms;
  for( int k = 0; k < 3; ++k )
  {
    in >> prms[k].initId;
  }
  for( int k = 0; k < 3; ++k )
  {
    in >> prms[k].log2windowsize;
  }
  for( int k = 0; k < 3; ++k )
  {
    in >> prms[k].adaptweight;
  }
  in >> prms[0].rateoffset[0] >> prms[0].rateoffset[1];
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
  in >> prms[1].rateoffset[0] >> prms[1].rateoffset[1];
  in >> prms[2].rateoffset[0] >> prms[2].rateoffset[1];
#else
  for( int k = 1; k < 3; ++k )
  {
    prms[k].rateoffset[0] = prms[0].rateoffset[0];
    prms[k].rateoffset[1] = prms[0].rateoffset[1];
  }
#endif
  in.ignore( 1024, '\n' );
  return prms;
}
#endif

static DataDb addDataFrame( istream &in, DataDb &db )
{
  char c = ( char ) in.peek();

  while( in&&c == 'c' )
  { 
    // loop seq
    std::string ctx;
    in >> ctx;
    if( ctx != "ctx" )
    {
      std::cerr << "[ERROR] invalid cabac file" << std::endl;
      exit( -1 );
    }
    int ctxidx;
    in >> ctxidx;

    if( db.ctxidx == -1 )
    {
      db.ctxidx = ctxidx;
      in.ignore( 1024, '\n' );
      db.modelsBPI = readModelParameters( in );
    }
    else if( ctxidx != db.ctxidx )
    {
      std::cerr << "[ERROR] mixinf different ctx idx" << std::endl;
      exit( -1 );
    }
    else
    {
      in.ignore( 1024, '\n' );
      readModelParameters( in );
    }

    DataSequence v;
    //in.ignore(1024, '\n');
    c = ( char ) in.peek();

    while( c == '#' )
    {
      in >> c;
      std::string ss;
      in >> ss;
      if( ss == "SIZE" )
      {
        in >> v.filesize;
      }
      in.ignore( 1024, '\n' );
      c = ( char ) in.peek();
    }

    DataFrame d;
    char slicetype;
    char reportslice;
    while( c != 'c' && in >> d.poc >> slicetype >> d.qp >> d.switchBp >> d.tempCABAC >> reportslice >> d.p0 >> d.p1 >> d.rate >> d.weight
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
            >> d.drate0>>d.drate1
#endif
           )
    { 
      // loop frames
      switch( slicetype )
      {
      case 'B':
        d.type = B_SLICE;
        break;
      case 'P':
        d.type = P_SLICE;
        break;
      case 'I':
        d.type = I_SLICE;
        break;
#if JVET_AH0176_LOW_DELAY_B_CTX
      case'L':
        d.type = L_SLICE;
        break;
#endif
      default:
        std::cerr << "[ERROR] unkwon slice type" << std::endl;
        exit( -1 );
      }

      switch( reportslice )
      {
      case 'B':
        d.reportslice = B_SLICE;
        break;
      case 'P':
        d.reportslice = P_SLICE;
        break;
      case 'I':
        d.reportslice = I_SLICE;
        break;
#if JVET_AH0176_LOW_DELAY_B_CTX
      case'L':
        d.reportslice = L_SLICE;
        break;
#endif
      default:
        std::cerr << "[ERROR] unkwon slice type" << std::endl;
        exit( -1 );
      }
      c = ( char ) in.get(); // get space
      c = ( char ) in.peek();

      if( c != '\n' )
      {
        std::string s;
        in >> s;
        d.bins.resize( s.size() );
        for( int k = 0; k < ( int ) s.size(); ++k )
        {
          d.bins[k] = (s[k] == '1');
        }
        v.v.push_back( std::move( d ) );
      }
      in.ignore( 1024, '\n' );
      c = ( char ) in.peek();
    }

    if( !v.v.empty() )
    {
      db.v.push_back( std::move( v ) );
    }
  }
  return db;
}

static DataDb loadDataFrame(std::ifstream& file)
{
  DataDb db;
  return addDataFrame( file, db );
}

static DataDb loadDataFrame()
{
  DataDb db;
  return addDataFrame( std::cin, db );
}

#if JVET_AH0176_LOW_DELAY_B_CTX
static void print(const Models &m)
{
  auto printField = [](const Models &m, int ModelParameters::* pi) 
  {
    for (int i = 0; i < (int)m.size(); ++i) 
    {
      std::cout << m[i].*pi << '\n';
    }
  };
  printField(m, &ModelParameters::initId);
  printField(m, &ModelParameters::log2windowsize);
  printField(m, &ModelParameters::adaptweight);
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
  const int nbRateOffset = (int)m.size();
#else
  const int nbRateOffset = 1;
#endif
  for (int i = 0; i < nbRateOffset; ++i) 
  {
    std::cout << m[i].rateoffset[0] << '\n';
    std::cout << m[i].rateoffset[1] << '\n';
  }
}
#else
static void print(const std::array<ModelParameters,3> &m)
{
  std::cout << m[0].initId << ",\n"
            << m[1].initId << ",\n"
            << m[2].initId << ",\n"
            << m[0].log2windowsize << ",\n"
            << m[1].log2windowsize << ",\n"
            << m[2].log2windowsize << ",\n"
            << m[0].adaptweight << ",\n"
            << m[1].adaptweight << ",\n"
            << m[2].adaptweight << ",\n"
            << m[0].rateoffset[0] << ",\n"
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
            << m[0].rateoffset[1] << ",\n"
            << m[1].rateoffset[0] << ",\n"
            << m[1].rateoffset[1] << ",\n"
            << m[2].rateoffset[0] << ",\n"
            << m[2].rateoffset[1] << "\n";
#else
            << m[0].rateoffset[1] << "\n";
#endif
}
#endif
