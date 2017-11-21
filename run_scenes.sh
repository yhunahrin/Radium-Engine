#!/bin/bash


# run a scene exporting subdivided meshes and anatomy
function run_scene_exportmesh()
{
    local infolder=$1
    local outfolder=$2
    
    source $infolder/scenefile

    DISPLAY=":0.0" \
    ./main-app --fps 30 --autoplay -e $outfolder -n $numframes \
    -f $infolder/$scenefile \    
    -c $infolder/$camfile \
    --anatfile $infolder/$anatfile \
    --kffile $infolder/$kffile  \
    --phyfile $infolder/$phyfile \
    --subdiv --showanat --exportmeshes --runanat 2>&1 >$outfolder/log.txt 
}

function run_scene_video()
{
    local infolder=$1
    local outfolder=$2
    
    source $infolder/scenefile

    DISPLAY=":0.0" \
    ./main-app --fps 30 --autoplay -e $outfolder -n $numframes \
    -f $infolder/$scenefile \    
    -c $infolder/$camfile \
    --anatfile $infolder/$anatfile \
    --kffile $infolder/$kffile  \
    --phyfile $infolder/$phyfile \
    --trans --showanat --exportframes --runanat 2>&1 >$outfolder/log.txt 
}

function make_video()
{
    local folder=$1
    local name=$2
    local fps=$3

    cd $folder
    # get the number of the first frame
    n=`ls | grep radiumframe | sort -n | head -n1 | tr -d -c '[0-9]'`
    ffmpeg -framerate $fps -start_number $n -i "radiumframe_%06d.png" -c:v libx264  $name.mp4

}

echo "Starting script"

run_scene_exportmesh "test1" "out1"
run_scene_exportmesh "test2" "out2"


