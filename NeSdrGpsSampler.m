%% 
%> @brief 
%>
%> @author Shu Wang
%> @param filename
%> 
close all ;
clear all ;

global GpsRx GpsTx

tic

GpsRx.CenterFrequency_Hz  = 1,575,420,000 ;
GpsRx.OversampleRate      = 1 ;
GpsRx.SampleRate_Hz       = 1,023,000 * 2 * GpsRx.OversampleRate ;
GpsRx.EnableTunerAGC      = false;
GpsRx.TunerGain_dB        = 100 ;
GpsRx.OutputDataType      = 'single' ;
GpsRx.SamplesPerFrame     = 256 * 1023 ;
GpsRx.StopTime_s          = GpsRx.SamplesPerFrame / GpsRx.SampleRate_Hz * 4 ;

GpsRx.radioFrameTime      = GpsRx.SamplesPerFrame / GpsRx.SampleRate_Hz ;

GpsRx.ADC.removeDC      = false ;
GpsRx.ADC.fixed         = false ;
GpsRx.ADC.magCount      = [0 0] ;
GpsRx.ADC.threshold     = [0 0] ;

GpsRx.radio = comm.SDRRTLReceiver('CenterFrequency', GpsRx.CenterFrequency_Hz, ...
                            'SampleRate', GpsRx.SampleRate_Hz, ...
                            'EnableTunerAGC', GpsRx.EnableTunerAGC , ...
                            'TunerGain', GpsRx.TunerGain_dB, ...
                            'SamplesPerFrame', GpsRx.SamplesPerFrame, ...
                            'OutputDataType', GpsRx.OutputDataType ) ;
isLocked( GpsRx.radio )
                        
GpsRx.sdrHwInfo = info( GpsRx.radio ) ;

GpsRx.sdrHwInfo
toc


%% Stream Processing Loop
%
% Capture GPS signals for 10 seconds which is specified by GpsRx.StopTime_ms.

%%
% Check for the status of the RTL-SDR radio
if ~isempty( sdrinfo( GpsRx.radio.RadioAddress ) )
    % Loop until the example reaches the simulation stop time
    timeCounter = 0;
    frameCount  = 0;
    while timeCounter < GpsRx.StopTime_s
        % Get baseband samples from RTL-SDR radio
        [x, ~] = step( GpsRx.radio );  % no 'len' output needed for blocking operation
        
        GpsRx.cData( (1:GpsRx.SamplesPerFrame) + GpsRx.SamplesPerFrame *  frameCount, 1 ) = x ;

        % Update counter
        timeCounter = timeCounter + GpsRx.radioFrameTime ;
        frameCount = frameCount + 1 ;
    end
else
    warning(message('SDR:sysobjdemos:MainLoop'))
end

%% 
% Release the audio and RTL-SDR resources.
release( GpsRx.radio ) ;
GpsRx.filename  = [GpsRx.sdrHwInfo.TunerName '_' num2str( round( now * 1e4 ) )] 

toc

GpsRx.dataLen   = length( GpsRx.cData ) ;
GpsRx.iqData    = [real( GpsRx.cData ) imag( GpsRx.cData )] ;


dlmwrite( [GpsRx.filename '.sdr'], GpsRx.cData ) ;
dlmwrite( [GpsRx.filename '.inq'], GpsRx.iqData, '\t' ) ;


%%
%> Check the quality of the received RF signals
%>
GpsRx.signal.mean   = [0 0] ;
GpsRx.signal.std    = [0 0] ;

GpsRx.signal.mean = mean( GpsRx.iqData ) ;

%%
%> remove DC if there is ANY. Most likely this is unecessary.
%>
if( GpsRx.ADC.removeDC )
    GpsRx.iqData(:, 1) = GpsRx.iqData(:, 1) - GpsRx.signal.mean(1) ;
    GpsRx.iqData(:, 2) = GpsRx.iqData(:, 2) - GpsRx.signal.mean(2) ;
end

%%
%>  check the standard deviation for checkig signal quality and setting 
%> the ADC threshold if necessary. 
%
GpsRx.signal.std = std( GpsRx.iqData ) ;

%%
%>  The ADC quantizes IF or ZF signals into 2-bit real digitalized IF 
%> or ZF data comprising MAG and SIGN components. The MAG values control 
%> the AGC loop, such that the MAG bit is active (HIGH) for approximately 
%> 33% of the time.
%>
%>  1-Sigma deviation is about 34.1 percent point of the normal 
%> distribution.
%>

if( false == GpsRx.ADC.fixed )      
    GpsRx.ADC.threshold = GpsRx.signal.std 
end


%%
%>  GPS RF signals will be digitalized and compressed into bytes: One I
%> or Q sample being compressed into two bits and two RF samples being 
%> compressed into one byte.
%
toc
for ii = 2:2:GpsRx.dataLen

    %% 
    %>  Convert and compress on two consecutive complex sample data into 
    %> one byte.
    %> 

    %%
    %> The expected ADC output is 2 bits, [ -3 -1 +1 +3 ] ;
    %>
    %> in = 0x01 = +1 --> ot = 0b0000 0000 = +0
    %> in = 0x03 = +3 --> ot = 0x0000 0010 = +2
    %> in = 0xFF = -1 --> ot = 0x0000 0001 = +1
    %> in = 0xFD = -3 --> ot = 0x0000 0011 = +3
    %>
    %> The percentage of the 2s and 3s in ADC outptus should be roughly 33
    %> percent.
    %>
    for jj = (ii-1):1:ii

        for IQ = 1:2

            %% ADC
            if( GpsRx.iqData( jj, IQ ) >= +GpsRx.ADC.threshold( IQ ) )

                GpsRx.adcData( jj*4-4 + IQ ) = int8( +3 ) ;
                GpsRx.adcData( jj*4-4 + IQ+2 ) = int8( +3 ) ;
                
                GpsRx.piqBuff( jj, IQ ) = uint8( 2 ) ;
                
                GpsRx.ADC.magCount( IQ ) = GpsRx.ADC.magCount( IQ ) + 1 ;

            elseif( GpsRx.iqData( jj, IQ ) >= 0 ) 

                GpsRx.adcData( jj*4-4 + IQ ) = int8( +1 ) ;
                GpsRx.adcData( jj*4-4 + IQ+2 ) = int8( +1 ) ;
                GpsRx.piqBuff( jj, IQ ) = uint8( 0 ) ;

            elseif( GpsRx.iqData( jj, IQ ) >= -GpsRx.ADC.threshold( IQ ) )

                GpsRx.adcData( jj*4-4 + IQ ) = int8( -1 ) ;
                GpsRx.adcData( jj*4-4 + IQ+2 ) = int8( -1 ) ;
                GpsRx.piqBuff( jj, IQ ) = uint8( 1 ) ;

            else

                GpsRx.adcData( jj*4-4 + IQ ) = int8( -3 ) ;
                GpsRx.adcData( jj*4-4 + IQ+2 ) = int8( -3 ) ;
                GpsRx.piqBuff( jj, IQ ) = uint8( 3 ) ;
                
                GpsRx.ADC.magCount( IQ ) = GpsRx.ADC.magCount( IQ ) + 1 ;

            end

        end
        
        %% Compress the first complex sample into a 4-bit or half-byte value
%         GpsRx.piqData( jj ) = uint8( bitxor( bitshift( GpsRx.adcData(jj, 1), 2, 'uint8' ), GpsRx.adcData(jj, 2) ) ) ;
%         GpsRx.piqData( jj ) = uint8( bitxor( bitshift( GpsRx.piqData( jj ), 4, 'uint8' ), GpsRx.piqData( jj ) ) ) ;   
        
        GpsRx.piqData( jj ) = uint8( ( GpsRx.piqBuff(jj, 1) * 4 + GpsRx.piqBuff(jj, 2) ) * 17 ) ;

    end
    
end

toc

GpsRx.piqFilename   = [GpsRx.filename '.piq'] ;
GpsRx.permission    = 'a' ;
GpsRx.machinefmt    = 'n' ;
GpsRx.piqFid        = fopen( GpsRx.piqFilename, GpsRx.permission, GpsRx.machinefmt ) ;   

fwrite( GpsRx.piqFid, GpsRx.piqData, 'uint8' ) ;
fclose( GpsRx.piqFid ) ;

GpsRx.adcFilename   = [GpsRx.filename '.bin'] ;
GpsRx.permission    = 'a' ;
GpsRx.machinefmt    = 'n' ;
GpsRx.adcFid        = fopen( GpsRx.adcFilename, GpsRx.permission, GpsRx.machinefmt ) ;   

fwrite( GpsRx.adcFid, GpsRx.adcData, 'int8' ) ;
fclose( GpsRx.adcFid ) ;

toc

GpsRx.Search.coherentMode   = 18 ;
GpsRx.Search.nonCoherentCnt = 20 ;
GpsRx.Search.coherentIntvl  = 18 ;

%%
%> run the native GpsRx GPS Searcher program.
%> 
%>  For a full search without assist, the GPS Searcher needs only the data
%> file as its input. 
%
command = ['./GpsSearcher2 ' GpsRx.adcFilename ...
            ' --ct ' int2str( GpsRx.Search.coherentMode ) ...
            ' --ci ' int2str( GpsRx.Search.coherentIntvl ) ] ;

%% Have memory issues? 
% Uncommon the instruction below to clean out more memory.
% clear ComTech GpsRx

switch computer
    
    case 'PCWIN64'
        [status, cmdout] = dos( command, '-echo' ) ;        
    
    otherwise
        [status, cmdout] = unix( command, '-echo' ) ;
end

    
    
    
    
    
    