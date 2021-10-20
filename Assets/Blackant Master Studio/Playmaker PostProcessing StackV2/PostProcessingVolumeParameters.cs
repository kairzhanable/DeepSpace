using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

namespace HutongGames.PlayMaker.Actions
{

	[ActionCategory("Post Processing Stack V2")]
	[Tooltip("Set parameter of post processing volume during runtime")]
	public class PostProcessingVolumeParameters : FsmStateAction
	{
		#region public variables
		[RequiredField]
		[ObjectType(typeof(PostProcessVolume))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmOwnerDefault ObjectVolume;
		
		[Tooltip("Check if effect is global")]
		public FsmBool IsGlobal;
		[Tooltip("Use this to smoothly Mix volume effect")]
		public FsmFloat BlendDistance;
		[HasFloatSlider(0,1)]
		[Tooltip("Use this to smoothly fuse volume effect")]
		public FsmFloat VolumeWeight;
		[Tooltip("set Volume priority")]
		public FsmInt VolumePriority;

		[ObjectType(typeof(PostProcessProfile))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmObject ObjectProfile;
		//public PostProcessProfile ObjectProfile;

		public FsmBool everyframe;

		#endregion

		// Code that runs on entering the state.
		public override void OnEnter()
		{
			SetVolumeParameters();
			if(!everyframe.Value) Finish();
		}

		public override void OnUpdate()
		{
			SetVolumeParameters();
		}

		void SetVolumeParameters()
		{
			if(ObjectVolume != null)
			{
				var go = Fsm.GetOwnerDefaultTarget(ObjectVolume);
				var UsedVolume = go.GetComponent<PostProcessVolume>();
				
				UsedVolume.isGlobal = IsGlobal.Value;
				if(!IsGlobal.Value)
				{
					UsedVolume.blendDistance = BlendDistance.Value;
				}
				UsedVolume.weight = VolumeWeight.Value;
				UsedVolume.priority = VolumePriority.Value;
				UsedVolume.profile = ObjectProfile. Value as PostProcessProfile;

			}
		}
	}

}
