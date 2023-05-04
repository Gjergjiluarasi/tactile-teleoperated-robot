import React from 'react'

function Button(props){
    return (
        <input 
        type='button' 
        value={props.caption}
        onClick={props.onClick}/>
    )
}

export default Button