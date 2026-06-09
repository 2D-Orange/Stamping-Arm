function [video, videoFile] = create_video_writer(videoBaseFile, quality)
%CREATE_VIDEO_WRITER Prefer MP4 and fall back to AVI when MPEG-4 is missing.

if nargin < 2 || isempty(quality)
    quality = 95;
end

try
    videoFile = [videoBaseFile, '.mp4'];
    video = VideoWriter(videoFile, 'MPEG-4');
catch
    videoFile = [videoBaseFile, '.avi'];
    video = VideoWriter(videoFile, 'Motion JPEG AVI');
end

if isprop(video, 'Quality')
    video.Quality = quality;
end
end
