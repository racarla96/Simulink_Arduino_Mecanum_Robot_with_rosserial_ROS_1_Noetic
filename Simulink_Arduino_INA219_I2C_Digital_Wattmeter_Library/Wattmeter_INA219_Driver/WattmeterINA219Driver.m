classdef WattmeterINA219Driver < realtime.internal.SourceSampleTime & ...
        coder.ExternalDependency
    %
    % System object template for a source block.
    % 
    % This template includes most, but not all, possible properties,
    % attributes, and methods that you can implement for a System object in
    % Simulink.
    %
    % NOTE: When renaming the class name Source, the file name and
    % constructor name must be updated to use the class name.
    %
    
    % Copyright 2016-2018 The MathWorks, Inc.
    %#codegen
    %#ok<*EMCA>
    
    properties     
        id = 0; % Unique ID {0-7}
        wire_number = 0; % Wire {0, 1}
        address = 45; % Address 0x{40, 41, 44, 45}
    end
    
    properties (Nontunable)
        % Public, non-tunable properties.
        
    end
    
    properties (Access = private)
    end
    
    methods
        % Constructor
        function obj = WattmeterINA219Driver(varargin)
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
                coder.cinclude('WattmeterINA219Driver.h');
                % coder.ceval('wINA219Driver_Init', int8(obj.id), int8(obj.wire_number), int8(obj.address));
                coder.ceval('wINA219Driver_Init', int8(obj.id), int8(obj.address));
            end
        end
        
        function [current_mA, voltage_mV]  = stepImpl(obj)   %#ok<MANU>
            current_mA = int32(0);
            voltage_mV = int32(0);
            if isempty(coder.target)
                % Place simulation output code here
            else
                % Call C-function implementing device output
                coder.ceval('wINA219Driver_Step', int8(obj.id), coder.wref(current_mA), coder.wref(voltage_mV));
            end
        end
        
        function releaseImpl(obj) %#ok<MANU>
            if isempty(coder.target)
                % Place simulation termination code here
            else
                % Call C-function implementing device termination
                %coder.ceval('wINA219Driver_Terminate');
            end
        end
    end
    
    methods (Access=protected)
        %% Define output properties
        function num = getNumInputsImpl(~)
            num = 0;
        end
        
        function num = getNumOutputsImpl(~)
            num = 2;
        end
        
        function varargout = isOutputFixedSizeImpl(~,~)
            varargout{1} = true;
            varargout{2} = true;
        end
        
        
        function varargout = isOutputComplexImpl(~)
            varargout{1} = false;
            varargout{2} = false;
        end
        
        function varargout = getOutputSizeImpl(~)
            varargout{1} = [1,1];
            varargout{2} = [1,1];
        end
        
        function varargout = getOutputDataTypeImpl(~)
            varargout{1} = 'int32';
            varargout{2} = 'int32';
        end
        
        function icon = getIconImpl(~)
            % Define a string as the icon for the System block in Simulink.
            icon = 'WattmeterINA219Driver';
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
            name = 'Source';
        end
        
        function b = isSupportedContext(context)
            b = context.isCodeGenTarget('rtw');
        end
        
        function updateBuildInfo(buildInfo, context)
            if context.isCodeGenTarget('rtw')
                % Update buildInfo
                srcDir = fullfile(fileparts(mfilename('fullpath')),'src'); 
                includeDir = fullfile(fileparts(mfilename('fullpath')),'include');
                libDir =  fullfile(fileparts(mfilename('fullpath')),'libraries');
                libDirA =  fullfile(fileparts(mfilename('fullpath')),'libraries/DFRobot_INA219');

                % Include header files
                addIncludePaths(buildInfo,includeDir);
                addIncludePaths(buildInfo,libDir);
                addIncludePaths(buildInfo,libDirA);

                % Include source files
                addSourceFiles(buildInfo,'WattmeterINA219Driver.cpp',srcDir);
                addSourceFiles(buildInfo,'DFRobot_INA219.cpp',libDirA);
    
                boardInfo = arduino.supportpkg.getBoardInfo;
        
                switch boardInfo.Architecture
                    case 'avr'
                        % Add SPI Library - For AVR Based
                        ideRootPath = arduino.supportpkg.getAVRRoot;
                        addIncludePaths(buildInfo, fullfile(ideRootPath, 'hardware', 'arduino', 'avr', 'libraries', 'SPI', 'src'));
                        srcFilePath = fullfile(ideRootPath, 'hardware', 'arduino', 'avr', 'libraries', 'SPI', 'src');
                        fileNameToAdd = {'SPI.cpp'};
                        addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
                
                        % Add Wire / I2C Library - For AVR Based
                        addIncludePaths(buildInfo, fullfile(ideRootPath, 'hardware', 'arduino', 'avr', 'libraries', 'Wire', 'src'));
                        addIncludePaths(buildInfo, fullfile(ideRootPath, 'hardware', 'arduino', 'avr', 'libraries', 'Wire', 'src', 'utility'));
                        srcFilePath = fullfile(ideRootPath, 'hardware', 'arduino', 'avr', 'libraries', 'Wire', 'src');
                        fileNameToAdd = {'Wire.cpp'};
                        addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
                        srcFilePath = fullfile(ideRootPath, 'hardware', 'arduino', 'avr', 'libraries', 'Wire', 'src', 'utility');
                        fileNameToAdd = {'twi.c'};
                        addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
                
                    case 'sam'
                        % Add SPI Library - For SAM Based
                        libSAMPath = arduino.supportpkg.getSAMLibraryRoot;
                        addIncludePaths(buildInfo, fullfile(libSAMPath, 'SPI','src'));
                        srcFilePath = fullfile(libSAMPath, 'SPI','src');
                        fileNameToAdd = {'SPI.cpp'};
                        addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
                
                        % Add Wire / I2C Library - For SAM Based
                        addIncludePaths(buildInfo, fullfile(libSAMPath, 'Wire', 'src'));
                        srcFilePath= fullfile(libSAMPath, 'Wire', 'src');
                        fileNameToAdd = {'Wire.cpp'};
                        addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
                
%                     case 'samd'
%                         % Add SPI Library - For SAMD Based
%                         libSAMDPath = arduino.supportpkg.getSAMDLibraryRoot;
%                         addIncludePaths(buildInfo, fullfile(libSAMDPath, 'SPI'));
%                         srcFilePath = fullfile(libSAMDPath, 'SPI');
%                         fileNameToAdd = {'SPI.cpp'};
%                         addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
%                 
%                         % Add Wire / I2C Library - For SAMD Based
%                         addIncludePaths(buildInfo, fullfile(libSAMDPath, 'Wire'));
%                         srcFilePath= fullfile(libSAMDPath, 'Wire');
%                         fileNameToAdd = {'Wire.cpp'};
%                         addSourceFiles(buildInfo, fileNameToAdd, srcFilePath);
                
                    otherwise
                        warning('Unexpected board type. Check again.')
                  end


%                 Use the following API's to add include files, sources and
%                 linker flags
%                 addIncludeFiles(buildInfo,'source.h',includeDir);
%                 addSourceFiles(buildInfo,'source.c',srcDir);
%                 addLinkFlags(buildInfo,{'-lSource'});
%                 addLinkObjects(buildInfo,'sourcelib.a',srcDir);
%                 addCompileFlags(buildInfo,{'-D_DEBUG=1'});
%                 addDefines(buildInfo,'MY_DEFINE_1')
            end
        end
    end
end
