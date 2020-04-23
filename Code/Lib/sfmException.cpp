/*=============================================================================

  PHAS0100ASSIGNMENT2: PHAS0100 Assignment 2 Social Force Model

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sfmException.h"

namespace sfm
{

//-----------------------------------------------------------------------------
Exception::Exception(const std::string& fileName,
                     int lineNumber)
: std::exception()
, m_Description("")
, m_FileName(fileName)
, m_LineNumber(lineNumber)
{
}


//-----------------------------------------------------------------------------
Exception::~Exception()
{

}


//-----------------------------------------------------------------------------
std::string Exception::GetFileName() const
{
  return m_FileName;
}


//-----------------------------------------------------------------------------
int Exception::GetLineNumber() const
{
  return m_LineNumber;
}


//-----------------------------------------------------------------------------
std::string Exception::GetDescription() const
{
  return m_Description;
}


//-----------------------------------------------------------------------------
void Exception::SetDescription(const std::string& desc)
{
  m_Description = desc;
}


//-----------------------------------------------------------------------------
const char* Exception::What()
{
  return m_Description.c_str();
}

} // end namespace
