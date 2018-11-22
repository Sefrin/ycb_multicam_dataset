Speed up video 5x and add text "5x":

```bash
ffmpeg -i 20181024_multicam_side.mp4 -vf "setpts=0.2*PTS, drawtext=fontfile=/usr/share/fonts/truetype/droid/DroidSans-Bold.ttf: \
text='5x': fontcolor=white: fontsize=48: box=1: boxcolor=black@0.5: \
boxborderw=5: x=(w-text_w)/10: y=(h-text_h)/10" 20181024_multicam_side_5x.mp4
```

Convert to gif:

```bash
ffmpeg -y -i 20181024_multicam_side_5x.mp4 -vf fps=10,scale=320:-1:flags=lanczos,palettegen palette.png

ffmpeg -i 20181024_multicam_side_5x.mp4 -i palette.png -filter_complex \
"fps=10,scale=320:-1:flags=lanczos[x];[x][1:v]paletteuse" 20181024_multicam_side_5x.gif
```

Convert to webm (see https://trac.ffmpeg.org/wiki/Encode/VP8 ):

```bash
ffmpeg -i 20181024_multicam_side_5x.mp4 -c:v libvpx -crf 7 -b:v 2M 20181024_multicam_side_5x.webm
```
