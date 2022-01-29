import sys
import os
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw
import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "-p",
    "--parent_folder",
    type=str,
    default="",
    help="parent folder",
)

arguments = parser.parse_args()
parent_folder = arguments.parent_folder
parent_folder_best = parent_folder + '_best_2.5'
parent_folder_merge = parent_folder + '_merge'


# # 44
# start_ind = 1760
# end_1_ind = 2360
# end_ind = 2500
# fusion_alg = 'DEFAULT'

# # 446
# start_ind = 1600
# end_1_ind = 1950
# end_ind = 2400
# fusion_alg = 'DEFAULT'

# # 259
# start_ind = 1150
# end_1_ind = 1500
# end_ind = 1950
# fusion_alg = 'MATHWORKS'

# # 29
start_ind = 560
end_1_ind = 980
end_ind = 1400
fusion_alg = 'MATHWORKS'

for i in range(start_ind, end_ind, 5):
    ind1 = ind2 = i
    if i > end_1_ind:
        ind1 = end_1_ind
    img_1 = parent_folder+'/front/'+str(ind1)+'.jpg'
    img_2 = parent_folder_best+'/front/'+str(ind2)+'.jpg'

    if os.path.exists(img_1) and os.path.exists(img_2):

        if not os.path.exists(parent_folder_merge):
            os.mkdir(parent_folder_merge)

        img_merge = parent_folder_merge+'/'+str(i)+'.jpg'



        img_1 = Image.open(img_1)
        img_2 = Image.open(img_2)

        images = [img_1, img_2]







        widths, heights = zip(*(i.size for i in images))

        total_width = sum(widths)
        max_height = max(heights)

        new_im = Image.new('RGB', (total_width, max_height))

        x_offset = 0
        for im in images:
          new_im.paste(im, (x_offset, 0))
          x_offset += im.size[0]

        font = ImageFont.truetype("MS Reference Sans Serif.ttf", 16)



        text1 = 'current fusion: ' + fusion_alg
        text2 = 'current fusion: ' + fusion_alg
        if i >= end_1_ind-250:
            text2 = 'current fusion: best_sensor'

        draw = ImageDraw.Draw(new_im)
        draw.text((0, 0),text1,(255,255,255),font=font)
        draw.text((291, 0),text2,(255,255,255),font=font)

        new_im.save(img_merge)
