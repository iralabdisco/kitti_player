function tracklets = readTracklets( filename )
% READTRACKLETS reads annotations from xml-files

%% read Document Object Model (DOM)
try
   dom = xmlread(filename); % document node (the complete tree)
catch
   error('Failed to read XML file %s.',filename);
end

%% Extract tracklets
allTracklets     = dom.getElementsByTagName('tracklets'); % DeepNodeList
trackletsElement = allTracklets.item(0); % DeferredElement

objIdx = 1; % object index

for i = 0:trackletsElement.getChildNodes.getLength-1 % all children of 'tracklets'

    element = trackletsElement.getChildNodes.item(i);
    attributes = element.getAttributes();

    if ~isempty(attributes)
        %fprintf('No attributes\n')
    
     %else
        %fprintf('found: %s\n',char(element.getTagName))

        if strcmp(element.getTagName,'count') % number of expected objects
            count = str2double(element.getFirstChild.getData);
            tracklets = cell(count,1); % init tracklets cell
            %fprintf( '%d\n', count );
        
        % print out version info
        % elseif strcmp(element.getTagName,'item_version') % meta-data
            % fprintf( '%d\n', str2double(element.getFirstChild.getData) );
        
        elseif strcmp(element.getTagName,'item') % a tracklet
            
            tracklet.objectType = ...
                char(element.getElementsByTagName('objectType').item(0).getTextContent);
            
            tracklet.h = str2double(element.getElementsByTagName('h').item(0).getTextContent);
            tracklet.w = str2double(element.getElementsByTagName('w').item(0).getTextContent);
            tracklet.l = str2double(element.getElementsByTagName('l').item(0).getTextContent);
            
            tracklet.first_frame = ...
                str2double(element.getElementsByTagName('first_frame').item(0).getTextContent);
            
            poses = element.getElementsByTagName('poses').item(0);
            tracklet.poses = extractPoses(poses);
            
            tracklet.finished = ... 
                str2double(element.getElementsByTagName('finished').item(0).getTextContent);
            
            % echoTracklet(tracklet);
            
            tracklets{objIdx} = tracklet;
            
            objIdx = objIdx+1;
        end
    end
end

% plausibility check
if count ~= objIdx-1
    fprintf(2,'number of tracklets (%d) does not match count (%d)!',objIdx,count);
end


end