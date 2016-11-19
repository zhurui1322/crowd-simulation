//
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

/// @file TestCasePlayerModule.cpp
/// @brief Implements the TestCasePlayerModule built-in module.

#include "modules/TestCasePlayerModule.h"
#include "testcaseio/TestCaseIO.h"
#include "util/Misc.h"
#include <iostream>
#include "obstacles/PolygonObstacle.h"
#include "obstacles/GJK_EPA.h"

using namespace SteerLib;

// #define _DEBUG 1

void TestCasePlayerModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo ) {
	_engine = engineInfo;
	_testCaseFilename = "";
	_aiModuleName = "";
	_aiModuleSearchPath = "";
	_aiModule = NULL;
	_obstacles.clear();

	// parse command line options
	SteerLib::OptionDictionary::const_iterator optionIter;
	for (optionIter = options.begin(); optionIter != options.end(); ++optionIter) {
		if ((*optionIter).first == "testcase") {
			_testCaseFilename = (*optionIter).second;
		}
		else if ((*optionIter).first == "ai") {
			_aiModuleName = (*optionIter).second;
		}
		else {
			throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to testCasePlayer module.");
		}
	}

	// load the module if it is not already
	if ( !_engine->isModuleLoaded(_aiModuleName) ) {
		_engine->loadModule(_aiModuleName,_aiModuleSearchPath,"");
		/// @todo add a boolean that flags this module was loaded here, so that we can unload the module after the simulation is done.
	}

	// get a pointer to the loaded module
	_aiModule = _engine->getModule(_aiModuleName);


#ifdef ENABLE_GUI
#ifdef ENABLE_QT

	QMainWindow * mainWindow = (QMainWindow*)(_engine->getEngineController()->getQtMainWindow());

	if(mainWindow != NULL) {

		/// @todo why is this tooltip attribute here, and not in the engine driver?
		mainWindow->QWidget::setAttribute(Qt::WA_AlwaysShowToolTips);

		_testCasePlayerDockWidget = new QDockWidget("Test Case Player");
		_testCasePlayerWidget = new SteerSimQt::TestCasePlayerWidget(this, _engine);
		_testCasePlayerDockWidget->setWidget(_testCasePlayerWidget);
		mainWindow->addDockWidget(Qt::RightDockWidgetArea, _testCasePlayerDockWidget);
	}
	else {
		_testCasePlayerDockWidget = NULL;
		_testCasePlayerWidget = NULL;
	}
#endif
#endif
}

void TestCasePlayerModule::initializeSimulation() {

	SteerLib::TestCaseReader * testCaseReader;

	std::string testCasePath;

	// try to find the test case in several ways:
	if (Util::fileCanBeOpened(_testCaseFilename)) {
		testCasePath = _testCaseFilename;
	}
	else if (Util::fileCanBeOpened( _engine->getTestCaseSearchPath() + _testCaseFilename )) {
		testCasePath = _engine->getTestCaseSearchPath() + _testCaseFilename;
	}
	else if (Util::fileCanBeOpened(_testCaseFilename + ".xml")) {
		testCasePath = _testCaseFilename + ".xml";
	}
	else if (Util::fileCanBeOpened( _engine->getTestCaseSearchPath() + _testCaseFilename + ".xml" )) {
		testCasePath = _engine->getTestCaseSearchPath() + _testCaseFilename + ".xml";
	}
	else {
		throw Util::GenericException("Could not find test case " + _testCaseFilename + ".");
	}

	// open the test case
	testCaseReader = new SteerLib::TestCaseReader();
	testCaseReader->readTestCaseFromFile(testCasePath);

	//Create the obstacles
	for (unsigned int i=0; i < testCaseReader->getNumObstacles(); i++) {
		const SteerLib::ObstacleInitialConditions * ic = testCaseReader->getObstacleInitialConditions(i);
		/*SteerLib::BoxObstacle * b;
		b = new SteerLib::BoxObstacle(ic.xmin, ic.xmax, ic.ymin, ic.ymax, ic.zmin, ic.zmax);*/
		SteerLib::ObstacleInterface *b = const_cast<SteerLib::ObstacleInitialConditions*>(ic)->createObstacle(); // TODO: FIX THIS.
			
			_obstacles.push_back(b);
			_engine->addObstacle(b);
			if(dynamic_cast<PolygonObstacle *>(b)){
			
			std::cout << "aobut to add polygon obstacle"<<std::endl;
			std::vector<Util::Vector> polygon;
			b->returnVertices(polygon);
			float cell_size = _engine->getSpatialDatabase()->getCellSizeX();
			std::cout<< " cell size  "<<cell_size<<std::endl;

			
			//unsigned int numCellsX = _engine->getSpatialDatabase()->getNumCellsX();
			//unsigned int numCellsZ = _engine->getSpatialDatabase()->getNumCellsZ();
			PolygonObstacle *po = dynamic_cast<PolygonObstacle *> (b);
			float r = (*po)._radius2;
			Util::Point p = po->position();
			Util::AxisAlignedBox box(p.x - r, p.x + r, 0.0f, 0.0f, p.z - r, p.z + r);
			std::cout<< " box x  "<<box.xmin<<std::endl;

		
		
			for(float i  = box.xmin; i < box.xmax  ; i += cell_size){
				for(float j = box.zmin ; j < box.zmax ; j += cell_size){
					// get index from point
					
					int ind = _engine->getSpatialDatabase()->getCellIndexFromLocation(i, j);
					std::cout<<"cell index "<<ind<<std::endl;
					//int indx, indz;
					//_engine->getSpatialDatabase()->getGridCoordinatesFromIndex(ind, indx, indz);
					Util::Point center;
					_engine->getSpatialDatabase()->getLocationFromIndex( ind, center);
					std::cout<<"cell center "<<center<<std::endl;
					std::vector<Util::Vector> cellShape;
					float adj = 0.0f;
					float cell_size_adj = adj + cell_size;
					cellShape.push_back(Util::Vector(center.x - cell_size_adj / 2.0f , 0.0f, center.z - cell_size_adj / 2.0f ));
       				cellShape.push_back(Util::Vector(center.x + cell_size_adj / 2.0f , 0.0f, center.z - cell_size_adj / 2.0f ));
       				cellShape.push_back(Util::Vector(center.x + cell_size_adj / 2.0f , 0.0f, center.z + cell_size_adj / 2.0f ));
       				cellShape.push_back(Util::Vector(center.x - cell_size_adj / 2.0f , 0.0f, center.z + cell_size_adj / 2.0f ));

       				std::vector<Util::Vector> simplex;

	       			if(SteerLib::GJK_EPA::GJK(polygon, cellShape, simplex)){ // polygon obstacle in range
	       				Util::AxisAlignedBox bd(center.x - cell_size/ 2.0f, center.x + cell_size/ 2.0f , 0.0f, 0.0f, center.z - cell_size/ 2.0f , center.z + cell_size / 2.0f);
	       				_engine->getSpatialDatabase()->addObject( b, bd);
	       				std::cout<<"bd.xmin "<<bd.xmin<<" bd.xmax "<<"bd.xmax "<<bd.xmax<<std::endl;
	       				std::cout<<"bd.zmin "<<bd.zmin<<" bd.zmax "<<"bd.zmax "<<bd.zmax<<std::endl;
	       				
	       			}
	       			
				}
			
			}


		}
		else{
		
		_engine->getSpatialDatabase()->addObject( b, b->getBounds());
		}
		
		

	
		// std::cout << "adding obstacle";
	}

	//Create the agents
	for (unsigned int i=0; i < testCaseReader->getNumAgents(); i++) {
		const SteerLib::AgentInitialConditions & ic = testCaseReader->getAgentInitialConditions(i);
		_engine->createAgent( ic, _aiModule );
	}

	//Modify camera targets
	Camera &engineCamera = _engine->getCamera();
	std::vector<Util::CurvePoint> controlPoints;
	std::vector<SteerLib::CameraView> cameraViews = testCaseReader->getCameraViews();
	Util::Vector startTangent(0.f, 0.f, 0.f);

	if ( cameraViews.size() != 0 )
		_engine->setCameraViewTestCase( cameraViews.front() );

	for (int i = 0; i < cameraViews.size(); i++)
		controlPoints.push_back(Util::CurvePoint(cameraViews[i].position, cameraViews[i].targetTangent, cameraViews[i].targetTime));
	controlPoints.push_back(Util::CurvePoint(engineCamera.position(), startTangent, 0.f));

	engineCamera.addControlPoints(controlPoints);

	delete testCaseReader;


#ifdef ENABLE_GUI
#ifdef ENABLE_QT
	if (_testCasePlayerWidget != NULL) _testCasePlayerWidget->setFrameNumber(_engine->getClock().getCurrentFrameNumber());
#endif
#endif

}

void TestCasePlayerModule::postprocessFrame(float timeStamp, float dt, unsigned int frameNumber) {
#ifdef ENABLE_GUI
#ifdef ENABLE_QT
	if (_testCasePlayerWidget != NULL) _testCasePlayerWidget->setFrameNumber(frameNumber);
#endif
#endif
}


void TestCasePlayerModule::cleanupSimulation() {
#ifdef _DEBUG
	std::cout << "about to TestCasePlayerModule::cleanupSimulation();\n";
#endif
	for (unsigned int i=0; i<_obstacles.size(); i++) {
		_engine->getSpatialDatabase()->removeObject( _obstacles[i], _obstacles[i]->getBounds());
		_engine->removeObstacle(_obstacles[i]);
		delete _obstacles[i];
	}
	_obstacles.clear();

	_engine->destroyAllAgentsFromModule(_aiModule);
}


void TestCasePlayerModule::finish() {

#ifdef ENABLE_GUI
#ifdef ENABLE_QT

	QMainWindow * mainWindow = (QMainWindow*)(_engine->getEngineController()->getQtMainWindow());
	if(mainWindow != NULL) {
		mainWindow->removeDockWidget(_testCasePlayerDockWidget);
		delete _testCasePlayerDockWidget;
		delete _testCasePlayerWidget; // may not be necessary, do Qt automatically take control and de-allocate children widgets?
	}

#endif
#endif

}
