function img = vsGetImageFromServer(t)
% t - the tcp connection established by tcpclient
% img - image in matlab format
%
% Function that returns an image obtained on a
% remote TCP/IP server, possibly in JPEG...
%
% vsantos, 2025
%-------------------------------------------------------------

img=NaN;
imgData = uint8([]);   % create empty data buffer to start with
% Send request to server
write(t, uint8('GET_IMAGE'));
% Read image size
sizeBytes = read(t, 4);
if numel(sizeBytes) < 4
    warning("Received incomplete size header");
    return;
end
imgSize = typecast(uint8(sizeBytes), 'uint32');
imgSize = swapbytes(imgSize);

% Read full image data
while length(imgData) < imgSize
    chunk = read(t, imgSize - length(imgData));
    if isempty(chunk)
        warning("Connection lost during image transfer");
        return;
    end
    imgData = [imgData; chunk];  %acumulate in case full image not at once
end
% Decode the raw into matlab matrix.
img=vsDecodeImage(imgData);

end  % of function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function img=vsDecodeImage(imgData)
% Decode JPEG using Java (in-memory)
import javax.imageio.ImageIO
import java.io.ByteArrayInputStream

byteStream = ByteArrayInputStream(imgData);
bufferedImage = ImageIO.read(byteStream);

if isempty(bufferedImage)
    warning("Could not decode image");
    return;
end

% Convert Java image to MATLAB array
h = bufferedImage.getHeight;
w = bufferedImage.getWidth;
pixels = typecast(bufferedImage.getData.getDataStorage(), 'uint8');
pixels = reshape(pixels, [3, w, h]);
img = permute(pixels, [3 2 1]);  % h x w x 3
img = img(:, :, [3 2 1]);       % BGR → RGB
end