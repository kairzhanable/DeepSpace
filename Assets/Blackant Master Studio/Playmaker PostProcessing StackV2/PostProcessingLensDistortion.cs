using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

namespace HutongGames.PlayMaker.Actions
{

	[ActionCategory("Post Processing Stack V2")]
	[Tooltip("Modify Lens Distortion During Runtime.")]
	public class PostProcessingLensDistortion : FsmStateAction
	{
		#region public variables
				
		[RequiredField]
		[ObjectType(typeof(PostProcessProfile))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmObject ObjectProfile;

		[Tooltip("Activate or deactivate Grain Layer")]
		public FsmBool ActiveLayer;
		[Tooltip("Activate or deactivate Grain Effect")]
		public FsmBool ActiveEffect;

		[Tooltip("Activate Intensity parameter")]
		public FsmBool IntensityActive;
		[HasFloatSlider(-250f, 250f)]
		[Tooltip("Lens Distortion Intensity")]
		public FsmFloat Intensity;

		[Tooltip("Activate Y Multiplier")]
		public FsmBool YMultiplierActive;
		[HasFloatSlider(0f, 10f)]
		[Tooltip("Y Multiplier")]
		public FsmFloat YMultiplier;

		[Tooltip("Activate X Multiplier")]
		public FsmBool XMultiplierActive;
		[HasFloatSlider(0f, 10f)]
		[Tooltip("X Multiplier")]
		public FsmFloat XMultiplier;

		[Tooltip("Activate X Center")]
		public FsmBool XCenterActive;
		[HasFloatSlider(-10f, 10f)]
		[Tooltip("X Center")]
		public FsmFloat XCenter;

		[Tooltip("Activate Y Center")]
		public FsmBool YCenterActive;
		[HasFloatSlider(-10f, 10f)]
		[Tooltip("Y Center")]
		public FsmFloat YCenter;

		[Tooltip("Activate Scale parameter")]
		public FsmBool ScaleActive;
		[HasFloatSlider(-5f, 5f)]
		[Tooltip("Scale Value")]
		public FsmFloat Scale;

		[Tooltip("Repeat every frame.")]
		public bool everyFrame;
		#endregion

		#region Private variables
		private LensDistortion LensLayer;
		private PostProcessProfile UsedProfile;		
		#endregion

		public override void Reset()
		{
			ActiveLayer= true;
			ActiveEffect = true;
			ObjectProfile = null;
			IntensityActive = true;
			Intensity = 0f;
			YMultiplierActive = true;
			YMultiplier = 1f;
			XMultiplierActive = true;
			XMultiplier = 1f;
			XCenterActive = true;
			XCenter = 0f;
			YCenterActive = true;
			YCenter = 0f;
			ScaleActive = true;
			Scale = 1f;
			everyFrame = false;
			
		}

		// Code that runs on entering the state.
		public override void OnEnter()
		{
			SetLensDistortion();
			if(!everyFrame)
			{
				Finish();
			}
		}

		// Code that runs every frame.
		public override void OnUpdate()
		{
			SetLensDistortion();
		}

		void SetLensDistortion()
		{
						
				UsedProfile = ObjectProfile.Value as PostProcessProfile;
				UsedProfile.TryGetSettings(out LensLayer);
				if(ObjectProfile != null & UsedProfile.HasSettings<LensDistortion>())
				{
					if(LensLayer!=null)
					{
						LensLayer.active = ActiveLayer.Value; //Active Lens Distortion Layer
						LensLayer.enabled.value = ActiveEffect.Value; // Active Lens Distortion Effect
						LensLayer.intensity.overrideState = IntensityActive.Value;//Activate intensity
						if(IntensityActive.Value)
						{ 	
							LensLayer.intensity.value = Intensity.Value; // intensity
						}
						LensLayer.intensityY.overrideState = YMultiplierActive.Value;//Activate Y intensity
						if(YMultiplierActive.Value)
						{ 	
							LensLayer.intensityY.value = YMultiplier.Value; // Y Intensity
						}
						LensLayer.intensityX.overrideState = XMultiplierActive.Value;//Activate X intensity
						if(XMultiplierActive.Value)
						{ 	
							LensLayer.intensityX.value = XMultiplier.Value; // X Intensity
						}
						LensLayer.centerX.overrideState = XCenterActive.Value;//Activate X Center
						if(XCenterActive.Value)
						{ 	
							LensLayer.centerX.value = XCenter.Value; // X center
						}
						LensLayer.centerY.overrideState = YCenterActive.Value;//Activate Y Center
						if(YCenterActive.Value)
						{ 	
							LensLayer.centerY.value = YCenter.Value; // Y center
						}
						LensLayer.scale.overrideState = ScaleActive.Value;//Activate Scale
						if(ScaleActive.Value)
						{ 	
							LensLayer.scale.value = Scale.Value; // Scale
						}
					}
				}
			}
		}
}


