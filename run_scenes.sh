#!/bin/bash


WORKDIR="/export/home/warhol/vroussel/code/git-clean/Radium-Engine/Bundle-GNU/RelWithDebInfo/bin"
SCRIPTDIR="`date +%Y%m%d-%H%M`"

# run a scene exporting subdivided meshes and anatomy
function run_scene_exportmesh()
{
    local infolder=$1
    local outfolder=$2
    local scenefile=$3

    source "$infolder/$scenefile"

    mkdir -p $outfolder

    DISPLAY=":0.0" \
    ./main-app --fps 30 --autoplay -e $outfolder -n $numframes \
    -f $infolder/$assetfile \
    -c $infolder/$camfile \
    --anatfile $infolder/$anatfile \
    --kffile $infolder/$kffile \
    --phyfile $infolder/$phyfile \
    --subdiv --showanat --savemeshes --runanat >& $outfolder/log.txt
}


# run a scene exporting frames with transparent skin for the video
function run_scene_video()
{
    local infolder=$1
    local outfolder=$2
    local scenefile=$3

    source $infolder/$scenefile

    mkdir -p $outfolder

    DISPLAY=":0.0" \
    ./main-app --fps 30 --autoplay -e $outfolder -n $numframes \
    -f $infolder/$assetfile \
    -c $infolder/$camfile \
    --anatfile $infolder/$anatfile \
    --kffile $infolder/$kffile \
    --phyfile $infolder/$phyfile \
    --trans --showanat --saveframes --runanat >& $outfolder/log.txt
}

# call ffmpeg on the exported frames
function make_video()
{
    local folder=$1
    local name=$2
    local fps=$3

    # get the number of the first frame
    local n=`ls $folder | grep radiumframe | sort -n | head -n1 | tr -d -c '[0-9]'`
    ffmpeg -framerate $fps -start_number $n -i "$folder/radiumframe_%06d.png" -c:v libx264  $folder/$name.mp4 >& log_ffmpeg.txt

}

function run_scene()
{
    local num=$1
    local in=$2
    local scene=$3

    local out="$SCRIPTDIR/scene$num"

    run_scene_exportmesh "$in" "$out/meshes" "$scene"
    run_scene_video "$in" "$out/frames" "$scene"
    make_video "$out/frames" "scene$num" "30"

    cp $in/desc.txt $out/meshes/scene.txt
}


echo "Starting script"
cd $WORKDIR

mkdir -p $SCRIPTDIR


echo "Running scene 0 (parameters presentation)"
run_scene "00" "/home/vroussel/Downloads/GEOM/arm_nomove" "scene00"
run_scene "01" "/home/vroussel/Downloads/GEOM/arm_nomove" "scene01"

echo "Running scene 1 (arm shake)"
run_scene 1 "/home/vroussel/Downloads/GEOM/arm_rise_shake" "scenefile"


echo "Running scene 2 (biceps curl)"
run_scene 2 "/home/vroussel/Downloads/GEOM/arm_biceps_curl" "scenefile"

rm -f list.txt
echo "Making full video"
find $SCRIPTDIR | grep mp4 | sort | while read file
do
    echo "file ' $file'" >> list.txt
done
ffmpeg -f concat -safe 0 -i list.txt -c copy fullvideo.mp4
mv fullvideo.mp4 $SCRIPTDIR

echo "Copying"
cp -a $SCRIPTDIR /mnt/STORM/wip/muscle-skinning/videos/source_files/


