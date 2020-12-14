//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#ifndef __STEERLIB_RECFILE_IO_H__
#define __STEERLIB_RECFILE_IO_H__

/// @file RecFileIO.h
/// @brief Defines the public interfaces for reading/writing rec files.

#include "Globals.h"
#include "Geometry.h"
#include "RecFileIOPrivate.h"

namespace SteerLib {

	//added by Cory, we're starting to have a lot of rec file formats running around
	enum REC_FORMAT {
		OPEN_ERROR,
		FORMAT_ERROR,
		STD_REC,	//this was put here for backwards compatibility
		FOOT_REC,
		DATA_REC,
		SHADOW_REC
	};

	/** 
	 * @brief The public interface for writing %SteerSuite rec files  (recordings of agents steering).
	 *
	 * Use this class to record agents steering through a scenario, creating a %SteerSuite rec file.
	 *
	 * <h3> How to use this class </h3>
	 * Using the RecFileWriter is straightforward:
	 *   -# Call startRecording(), specifying the number of agents and the filename
	 *      of the rec file you wish to create.
	 *       - If you plan to benchmark the recording, you should specify the test case name that
	 *         is being recorded.
	 *   -# To write the next frame of the simulation, call startFrame(), and then call
	 *      setAgentInfoForCurrentFrame() for all agents, and then call finishFrame().
	 *       - <i><b>setAgentInfoForCurrentFrame() should be called for all agents, even if they are
	 *         inactive or disabled. Failing to do so may result in undefined behavior.</b></i>
	 *       - If you plan to benchmark the recording, the very first recorded frame should be the initial
	 *         conditions of the test case.
	 *   -# To finish recording the simulation, call finishRecording().
	 *
	 * <h3> Notes </h3>
	 *
	 * This class enforces correct calling sequence by throwing exceptions if functions are called at
	 * the wrong time.  For example, code cannot call startFrame() if the previous frame was not 
	 * finished with finishFrame().  It is recommended to catch these sorts of exceptions and to look at 
	 * the exception::what() message string.
	 *
	 * @see 
	 *  - SteerLib::RecFileReader to read rec files
	 *  - Util::GenericException class for an example of catching the exceptions and printing the useful error message.
	 */
	class STEERLIB_API RecFileWriter : public RecFileWriterPrivate {
	public:
		/// @name Constructors and destructors
		//@{
		RecFileWriter();
		~RecFileWriter();
		//@}

		/// @name Meta data queries
		//@{
		/// Returns the filename being written to
		const std::string & getFilename() { return _filename; }
		/// Returns the version being written
		unsigned int getVersion() { return _version; }
		/// Returns true if the RecFileWriter has a recfile open; this is true between startRecording() and finishRecording() calls.
		bool isOpen() { return _opened; }
		/// Returns true if the RecFileWriter has a recfile open; this is true between startRecording() and finishRecording() calls.
		bool isRecording() { return _opened; }
		/// Returns true if the RecFileWriter is currently writing a frame; this is true between startFrame() and finishFrame() calls.
		bool isWritingFrame() { return _writingFrame; }
		//@}

		/// @name Operations to write the rec file
		//@{
		/// Starts a new rec file to be recorded, optionally associated with a test case name; if you intend to benchmark this recording, you should provide the associated test case name.
		void startRecording(size_t numAgents, const std::string & filename, const std::string & testCaseName = "");
		/// Finishes a recording of a rec file.
		void finishRecording();
		/// Starts a new frame in the rec file to be recorded.
		void startFrame( float timeStamp, float timePassedSinceLastFrame );
		/// Finishes the current frame being recorded.
		void finishFrame();

		/// Sets the agent's info for the frame that is currently being recorded, must be called between startFrame() and finishFrame();
		void setAgentInfoForCurrentFrame( unsigned int agentIndex, float posx, float posy, float posz, float dirx, float diry, float dirz, float goalx, float goaly, float goalz, float radius, bool enabled );
		/// Sets the agent's info for the frame that is currently being recorded, must be called between startFrame() and finishFrame();
		inline void setAgentInfoForCurrentFrame( unsigned int agentIndex, const Util::Point & pos, const Util::Vector & dir, const Util::Point & goal, float radius, bool enabled ) { setAgentInfoForCurrentFrame(agentIndex, pos.x, pos.y, pos.z, dir.x, dir.y, dir.z, goal.x, goal.y, goal.z, radius, enabled); }

		/// Adds an obstacle's info to the recording.
		void addObstacleBoundingBox( float xmin, float xmax, float ymin, float ymax, float zmin, float zmax );
		/// Adds an obstacle's info to the recording.
		inline void addObstacleBoundingBox( const Util::AxisAlignedBox & bb ) { addObstacleBoundingBox(bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax); }

		/// Adds a suggested camera view to the recording.
		void addCameraView( float origx, float origy, float origz, float lookatx, float lookaty, float lookatz);
		/// Adds a suggested camera view to the recording.
		inline void addCameraView( const Util::Point & pos, const Util::Point & lookat ) { addCameraView( pos.x, pos.y, pos.z, lookat.x, lookat.y, lookat.z ); }
		//@}
	};


} // end namespace SteerLib

#endif
