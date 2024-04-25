classdef Robot_Mecanum_rosserial_Driver < realtime.internal.SourceSampleTime ...
        & coder.ExternalDependency 
    
    % Block to communicate data to communicate data to ROS 1 Distribution

    % Copyright 2016-2018 The MathWorks, Inc.
    %#codegen
    %#ok<*EMCA>
    
    properties

    end
    
    properties (Nontunable)
        %Serial (1,1) int {mustBeMember(Serial, [0,1,2,3])} = 0;
    end
    
    properties (Access = private)
        % Pre-computed constants.
    end

    properties (Access = protected)

    end
    
    methods
        % Constructor
        function obj = Robot_Mecanum_rosserial_Driver(varargin)
            % Support name-value pair arguments when constructing the object.
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods (Access=protected)
        function setupImpl(obj) %#ok<MANU>
            if isempty(coder.target)
                % Place simulation setup code here
            else
                % Call C-function implementing device initialization
                coder.cinclude('Robot_Mecanum_rosserial_Driver.h');
                coder.ceval('Robot_Mecanum_rosserial_Driver_Init');
            end
        end
        
        function [ref_w1, ref_w2, ref_w3, ref_w4] = stepImpl(~, a_x, a_y, w_z, theta, w1, w2, w3, w4, V, I)   %#ok<MANU>
            if isempty(coder.target)
                % Place simulation output code here
            else
                ref_w1 = single(0);
                ref_w2 = single(0);
                ref_w3 = single(0);
                ref_w4 = single(0);

                % Call C-function implementing device output
                coder.ceval('Robot_Mecanum_rosserial_Driver_Step', coder.wref(ref_w1), coder.wref(ref_w2), coder.wref(ref_w3), coder.wref(ref_w4), a_x, a_y, w_z, theta, w1, w2, w3, w4, V, I);
            end
        end
        
        function releaseImpl(obj) %#ok<MANU>
            if isempty(coder.target)
                % Place simulation termination code here
            else
                % Call C-function implementing device termination
                %coder.ceval('source_terminate');
            end
        end
    end
    
    methods (Access=protected)
        %% Define output properties
        function num = getNumInputsImpl(~)
            num = 10;
        end
        
        function num = getNumOutputsImpl(~)
            num = 4;
        end
        
        function flag = isInputSizeMutableImpl(~,~)
            flag = false;
        end
        
        function flag = isInputComplexityMutableImpl(~,~)
            flag = false;
        end
        
        function validateInputsImpl(~, a_x, a_y, w_z, theta, w1, w2, w3, w4, V, I)
            if isempty(coder.target)
                % Run input validation only in Simulation
                validateattributes(a_x,{'single'},{'scalar'},'','a_x');
                validateattributes(a_y,{'single'},{'scalar'},'','a_y');
                validateattributes(w_z,{'single'},{'scalar'},'','w_z');
                validateattributes(theta,{'single'},{'scalar'},'','theta');
                validateattributes(w1,{'single'},{'scalar'},'','w1');
                validateattributes(w2,{'single'},{'scalar'},'','w2');
                validateattributes(w3,{'single'},{'scalar'},'','w3');
                validateattributes(w4,{'single'},{'scalar'},'','w4');
                validateattributes(V,{'single'},{'scalar'},'','V');
                validateattributes(I,{'single'},{'scalar'},'','I');
            end
        end
        
        function icon = getIconImpl(~)
            % Define a string as the icon for the System block in Simulink.
            icon = 'Robot_Mecanum_rosserial_Driver';
        end    
    end
    
    methods (Static, Access=protected)
        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end
        
        function isVisible = showSimulateUsingImpl
            isVisible = false;
        end
    end
    
    methods (Static)
        function name = getDescriptiveName()
            name = 'Robot_Mecanum_rosserial_Driver';
        end
        
        function b = isSupportedContext(context)
            b = context.isCodeGenTarget('rtw');
        end
        
        function updateBuildInfo(buildInfo, context)
            if context.isCodeGenTarget('rtw')
                % Update buildInfo
                srcDir = fullfile(fileparts(mfilename('fullpath')),'src'); %#ok<NASGU>
                includeDir = fullfile(fileparts(mfilename('fullpath')),'include');
                libDir =  fullfile(fileparts(mfilename('fullpath')),'libraries');
                libDirA =  fullfile(fileparts(mfilename('fullpath')),'libraries/ros_lib');
                %libDirA_ros =  fullfile(fileparts(mfilename('fullpath')),'libraries/ros_lib/ros');

                % Include header files
                addIncludePaths(buildInfo,includeDir);
                addIncludePaths(buildInfo,libDir);
                addIncludePaths(buildInfo,libDirA);
                %addIncludePaths(buildInfo,libDirA_ros);

                % Include source files
                addSourceFiles(buildInfo,'Robot_Mecanum_rosserial_Driver.cpp',srcDir);
                addSourceFiles(buildInfo,'time.cpp',libDirA);

                % boardInfo = arduino.supportpkg.getBoardInfo;
                % 
                % switch boardInfo.Architecture
                %     case 'avr'
                %         % Add SPI Library - For AVR Based
                %         ideRootPath = arduino.supportpkg.getAVRRoot;
                %         addIncludePaths(buildInfo, fullfile(ideRootPath, 'hardware', 'arduino', 'avr', 'libraries', 'SPI', 'src'));
                %         srcFilePath = fullfile(ideRootPath, 'hardware', 'arduino', 'avr', 'libraries', 'SPI', 'src');
                %         fileNameToAdd = {'SPI.cpp'};
                %         addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
                % 
                %         % Add Wire / I2C Library - For AVR Based
                %         addIncludePaths(buildInfo, fullfile(ideRootPath, 'hardware', 'arduino', 'avr', 'libraries', 'Wire', 'src'));
                %         addIncludePaths(buildInfo, fullfile(ideRootPath, 'hardware', 'arduino', 'avr', 'libraries', 'Wire', 'src', 'utility'));
                %         srcFilePath = fullfile(ideRootPath, 'hardware', 'arduino', 'avr', 'libraries', 'Wire', 'src');
                %         fileNameToAdd = {'Wire.cpp'};
                %         addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
                %         srcFilePath = fullfile(ideRootPath, 'hardware', 'arduino', 'avr', 'libraries', 'Wire', 'src', 'utility');
                %         fileNameToAdd = {'twi.c'};
                %         addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
                % 
                %         % Include source files
                %         libDirA_arch =  fullfile(fileparts(mfilename('fullpath')),'libraries/Servo-1.2.1/src/avr');
                %         addSourceFiles(buildInfo,'Servo.cpp',libDirA_arch);
                % 
                %     case 'sam'
                %         % Add SPI Library - For SAM Based
                %         libSAMPath = arduino.supportpkg.getSAMLibraryRoot;
                %         addIncludePaths(buildInfo, fullfile(libSAMPath, 'SPI','src'));
                %         srcFilePath = fullfile(libSAMPath, 'SPI','src');
                %         fileNameToAdd = {'SPI.cpp'};
                %         addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
                % 
                %         % Add Wire / I2C Library - For SAM Based
                %         addIncludePaths(buildInfo, fullfile(libSAMPath, 'Wire', 'src'));
                %         srcFilePath= fullfile(libSAMPath, 'Wire', 'src');
                %         fileNameToAdd = {'Wire.cpp'};
                %         addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
                % 
                %         % Include source files
                %         libDirA_arch =  fullfile(fileparts(mfilename('fullpath')),'libraries/Servo-1.2.1/src/sam');
                %         addSourceFiles(buildInfo,'Servo.cpp',libDirA_arch);
                
    %                 case 'samd'
    %                     % Add SPI Library - For SAMD Based
    %                     libSAMDPath = arduino.supportpkg.getSAMDLibraryRoot;
    %                     addIncludePaths(buildInfo, fullfile(libSAMDPath, 'SPI'));
    %                     srcFilePath = fullfile(libSAMDPath, 'SPI');
    %                     fileNameToAdd = {'SPI.cpp'};
    %                     addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
    %             
    %                     % Add Wire / I2C Library - For SAMD Based
    %                     addIncludePaths(buildInfo, fullfile(libSAMDPath, 'Wire'));
    %                     srcFilePath= fullfile(libSAMDPath, 'Wire');
    %                     fileNameToAdd = {'Wire.cpp'};
    %                     addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
                
                    % otherwise
                    %     warning('Unexpected board type. Check again.')
                    % end


                % Use the following API's to add include files, sources and
                % linker flags
                %addIncludeFiles(buildInfo,'source.h',includeDir);
                %addSourceFiles(buildInfo,'source.c',srcDir);
                %addLinkFlags(buildInfo,{'-lSource'});
                %addLinkObjects(buildInfo,'sourcelib.a',srcDir);
                %addCompileFlags(buildInfo,{'-D_DEBUG=1'});
                %addDefines(buildInfo,'MY_DEFINE_1')
            end
        end
    end
end
