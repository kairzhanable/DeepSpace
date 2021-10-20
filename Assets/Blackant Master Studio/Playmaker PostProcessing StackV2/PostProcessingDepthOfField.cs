using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

namespace HutongGames.PlayMaker.Actions
{

	[ActionCategory("Post Processing Stack V2")]
	[Tooltip("Modify Depth of Field during runtime.")]
	public class PostProcessingDepthOfField : FsmStateAction
	{
		#region public variables
				
		[RequiredField]
		[ObjectType(typeof(PostProcessProfile))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmObject ObjectProfile;

		[Tooltip("Activate or deactivate Dof Layer")]
		public FsmBool ActiveLayer;
		[Tooltip("Activate or deactivate AO Effect")]
		public FsmBool ActiveEffect;

		[Tooltip("Activate or deactivate Focus Distance Parameter")]
		public FsmBool FocusDistanceActive;
		[Tooltip("Focus Distance")]
		public FsmFloat FocusDistance;

		[Tooltip("Activate or deactivate Aperture parameter")]
		public FsmBool ApertureActive;
		[Tooltip("Intensity of Aperture")]
		public FsmFloat Aperture;

		[Tooltip("Activate or deactivate Focal Length parameter")]
		public FsmBool FocalActive;
		[Tooltip("Focal Length")]
		public FsmInt FocalLength;

		[Tooltip("Activate or deactivate Max Blur parameter")]
		public FsmBool MaxBlurActive;
		[Tooltip("Max Blur Size")]
		public KernelSize MaxBlur;

		[Tooltip("Repeat every frame.")]
		public bool everyFrame;
		#endregion

		#region Private variables
		private DepthOfField DofLayer;
		private PostProcessProfile UsedProfile;		
		#endregion


		public override void Reset()
		{
			ActiveLayer= true;
			ActiveEffect = true;
			ObjectProfile = null;
			FocusDistanceActive = true;
			FocusDistance = 0f;
			ApertureActive = true;
			Aperture = 0f;
			FocalActive = true;
			FocalLength = 0;
			MaxBlurActive= true;
			MaxBlur = 0;
			everyFrame = false;
		}

		// Code that runs on entering the state.
		public override void OnEnter()
		{
			SetDoF();
			if(!everyFrame)
			{
				Finish();
			}
		}

		// Code that runs every frame.
		public override void OnUpdate()
		{
			SetDoF();
		}

		void SetDoF()
		{
			
				
				UsedProfile = ObjectProfile.Value as PostProcessProfile;
				UsedProfile.TryGetSettings(out DofLayer);
				if(ObjectProfile != null & UsedProfile.HasSettings<DepthOfField>())
				{
					if(DofLayer!=null)
					{
						DofLayer.active = ActiveLayer.Value; //Active AO Layer
						DofLayer.enabled.value = ActiveEffect.Value; // Active AO Effect
						DofLayer.focusDistance.overrideState = FocusDistanceActive.Value ;//Activate Focus Distance
						if(FocusDistanceActive.Value)
						{
							DofLayer.focusDistance.value = FocusDistance.Value ; // set distance value
						}
						DofLayer.aperture.overrideState = ApertureActive.Value ;//Activate aperture
						if(ApertureActive.Value)
						{
							DofLayer.aperture.value = Aperture.Value ; // set aperture value
						}
						DofLayer.focalLength.overrideState = FocalActive.Value ;//Activate focal length
						if(FocalActive.Value)
						{
							DofLayer.focalLength.value = FocalLength.Value ; // set focal value
						}
						DofLayer.kernelSize.overrideState = MaxBlurActive.Value ;//Activate Max Blur Size
						if(MaxBlurActive.Value)
						{
							DofLayer.kernelSize.value = MaxBlur ; // set Max Blur parameter
						}


					}
				}
			}
		}
}


